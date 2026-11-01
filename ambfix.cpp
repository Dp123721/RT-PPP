// Implements wide-lane and narrow-lane LAMBDA ambiguity fixing.
#include "ambfix.h"

#include "gnss_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kPi = 3.1415926535897932384626433832795;
} // namespace

namespace AmbFix {

namespace {

struct LambdaCandidate {
    Vec fixed;
    double distSearch = 0.0;
    double distOriginal = 0.0;
};

struct LambdaDebug {
    int nCan = 0;
    int kept = 0;
    double chi = 0.0;
    double dist1 = 0.0;
    double dist2 = 0.0;
    double ratioNumerator = 0.0;
    double ratioDenominator = 0.0;
    QString ratioMode;
    QString failReason;
    QVector<LambdaCandidate> candidates;
};

struct AttemptEval {
    bool lambdaOk = false;
    bool accepted = false;
    QString rejectReason;
    Vec fixed;
    double ratio = 0.0;
    double ratioRaw = 0.0;
    double boot = 0.0;
    double maxNlDev = 0.0;
    int worstLocal = -1;
    QString worstDd;
    double worstQdd = 0.0;
    double qddMin = 0.0;
    double qddMax = 0.0;
    double qddAvg = 0.0;
    LambdaDebug lambdaDebug;
};

Mat zeros(int r, int c) {
    return Mat(r, Vec(c, 0.0));
}

Mat transpose(const Mat& a) {
    if (a.isEmpty()) return Mat();
    Mat out = zeros(a[0].size(), a.size());
    for (int i = 0; i < a.size(); ++i) {
        for (int j = 0; j < a[0].size(); ++j) {
            out[j][i] = a[i][j];
        }
    }
    return out;
}

Vec multiply(const Mat& a, const Vec& v) {
    Vec out(a.size(), 0.0);
    for (int i = 0; i < a.size(); ++i) {
        double sum = 0.0;
        for (int j = 0; j < v.size(); ++j) {
            sum += a[i][j] * v[j];
        }
        out[i] = sum;
    }
    return out;
}

Mat multiply(const Mat& a, const Mat& b) {
    if (a.isEmpty() || b.isEmpty()) return Mat();
    Mat out = zeros(a.size(), b[0].size());
    for (int i = 0; i < a.size(); ++i) {
        for (int k = 0; k < b.size(); ++k) {
            const double aik = a[i][k];
            if (aik == 0.0) continue;
            for (int j = 0; j < b[0].size(); ++j) {
                out[i][j] += aik * b[k][j];
            }
        }
    }
    return out;
}

bool invert(const Mat& a, Mat& inv) {
    const int n = a.size();
    if (n == 0 || a[0].size() != n) {
        return false;
    }
    Mat aug = zeros(n, 2 * n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            aug[i][j] = a[i][j];
        }
        aug[i][n + i] = 1.0;
    }
    for (int i = 0; i < n; ++i) {
        int pivot = i;
        double maxVal = std::fabs(aug[i][i]);
        for (int r = i + 1; r < n; ++r) {
            double val = std::fabs(aug[r][i]);
            if (val > maxVal) {
                maxVal = val;
                pivot = r;
            }
        }
        if (maxVal < 1e-12) {
            return false;
        }
        if (pivot != i) {
            std::swap(aug[i], aug[pivot]);
        }
        double diag = aug[i][i];
        for (int c = 0; c < 2 * n; ++c) {
            aug[i][c] /= diag;
        }
        for (int r = 0; r < n; ++r) {
            if (r == i) continue;
            double factor = aug[r][i];
            if (factor == 0.0) continue;
            for (int c = 0; c < 2 * n; ++c) {
                aug[r][c] -= factor * aug[i][c];
            }
        }
    }
    inv = zeros(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            inv[i][j] = aug[i][n + j];
        }
    }
    return true;
}

double quadraticDistance(const Mat& Q, const Vec& a, const Vec& fixed, bool& ok) {
    ok = false;
    if (a.size() != fixed.size() || Q.size() != a.size()) {
        return 0.0;
    }
    Mat Qinv;
    if (!invert(Q, Qinv)) {
        return 0.0;
    }
    Vec diff(a.size(), 0.0);
    for (int i = 0; i < a.size(); ++i) {
        diff[i] = a[i] - fixed[i];
    }
    Vec qd = multiply(Qinv, diff);
    double dist = 0.0;
    for (int i = 0; i < diff.size(); ++i) {
        dist += diff[i] * qd[i];
    }
    ok = std::isfinite(dist);
    return dist;
}

Mat identity(int n) {
    Mat out = zeros(n, n);
    for (int i = 0; i < n; ++i) {
        out[i][i] = 1.0;
    }
    return out;
}

Mat subMatrix(const Mat& in, const QVector<int>& keep) {
    Mat out = zeros(keep.size(), keep.size());
    for (int i = 0; i < keep.size(); ++i) {
        for (int j = 0; j < keep.size(); ++j) {
            out[i][j] = in[keep[i]][keep[j]];
        }
    }
    return out;
}

Vec subVector(const Vec& in, const QVector<int>& keep) {
    Vec out(keep.size(), 0.0);
    for (int i = 0; i < keep.size(); ++i) {
        out[i] = in[keep[i]];
    }
    return out;
}

QString joinDds(const QVector<QString>& ddNames, const QVector<int>& indices) {
    QStringList parts;
    for (int idx : indices) {
        if (idx >= 0 && idx < ddNames.size()) {
            parts << ddNames[idx];
        }
    }
    return parts.isEmpty() ? QStringLiteral("-") : parts.join(QLatin1Char(','));
}

bool hasStrongHistory(int fixEpochs, int lockEpochs, const Options& opt) {
    if (!opt.useHistoryAid) {
        return false;
    }
    return fixEpochs >= opt.histMinFixEpochs && lockEpochs >= opt.histMinLockEpochs;
}

bool ddHasStrongWlHistory(const Sat& sat, const Sat& ref, const Options& opt) {
    return hasStrongHistory(sat.wlFixEpochs, sat.lockEpochs, opt) &&
           hasStrongHistory(ref.wlFixEpochs, ref.lockEpochs, opt);
}

bool ddHasStrongNlHistory(const Sat& sat, const Sat& ref, const Options& opt) {
    return hasStrongHistory(sat.nlFixEpochs, sat.lockEpochs, opt) &&
           hasStrongHistory(ref.nlFixEpochs, ref.lockEpochs, opt);
}

bool ddIsWarmup(const Sat& sat, const Sat& ref, const Options& opt) {
    if (!opt.useHistoryAid) {
        return false;
    }
    return sat.newArc || ref.newArc ||
           sat.nlFixEpochs == 0 || ref.nlFixEpochs == 0 ||
           sat.lockEpochs < opt.partialWarmupLockEpochs ||
           ref.lockEpochs < opt.partialWarmupLockEpochs;
}

using Vec1 = QVector<double>;
using Mat1 = QVector<Vec1>;

Mat1 zeros1(int n) {
    return Mat1(n + 1, Vec1(n + 1, 0.0));
}

double normalCdf(double x) {
    return 0.5 * std::erfc(-x / std::sqrt(2.0));
}

double bootStrapProb(double sigma) {
    if (sigma >= 0.3) {
        return 0.9;
    }
    if (sigma <= 0.07) {
        return 1.0;
    }
    const double dt = (sigma - 0.07) * 2000.0;
    const int i = static_cast<int>(dt);
    const int j = i + 1;
    const double s0 = 0.07 + 0.0005 * i;
    const double s1 = 0.07 + 0.0005 * j;
    const double v0 = 2.0 * normalCdf(0.5 / s0) - 1.0;
    const double v1 = 2.0 * normalCdf(0.5 / s1) - 1.0;
    return (j - dt) * v0 + (dt - i) * v1;
}

void backTrack(int n, int& idx, Vec1& end, Vec1& dist, Vec1& lef, Vec1& left, bool& ended) {
    for (idx = idx + 1; idx <= n; ++idx) {
        if (dist[idx] <= end[idx]) {
            dist[idx] += 1.0;
            left[idx] = std::pow(dist[idx] + lef[idx], 2);
            return;
        }
    }
    ended = true;
}

void storeCand(int can, int pos, int& maxPos, double t, double& tMax,
               const Vec1& dist, Vec1& disall, Mat1& cands, int n) {
    for (int i = 1; i <= n; ++i) {
        cands[i][pos] = dist[i];
    }
    disall[pos] = t;
    tMax = t;
    maxPos = pos;
    for (int i = 1; i <= can; ++i) {
        if (disall[i] > tMax) {
            maxPos = i;
            tMax = disall[i];
        }
    }
}

void collectCand(int n, int maxCan, double dD1, double dLef1, double dLeft1, double dRight1,
                 double dChi, Vec1& dist, double& end1, int& ncan, Vec1& disall,
                 Mat1& cands, double& tMax, int& maxPos) {
    double t = dChi - (dRight1 - dLeft1) * dD1;
    end1 += 1.0;
    while (true) {
        ++ncan;
        if (ncan >= 10000) {
            break;
        }
        if (ncan <= maxCan) {
            storeCand(ncan, ncan, maxPos, t, tMax, dist, disall, cands, n);
        } else if (t < tMax) {
            storeCand(maxCan, maxPos, maxPos, t, tMax, dist, disall, cands, n);
        }
        t += (2.0 * (dist[1] + dLef1) + 1.0) * dD1;
        dist[1] += 1.0;
        if (dist[1] > end1) {
            break;
        }
    }
}

int jnt2(double d) {
    if (d <= 0.0) {
        return static_cast<int>(std::ceil(d));
    }
    return static_cast<int>(std::floor(d) + 1.0);
}

double dinki(int n, const Mat1& L, const Vec1& vec, int col) {
    double out = 0.0;
    for (int i = col + 1; i <= n; ++i) {
        out += L[i][col] * vec[i];
    }
    return out;
}

int fmfac(int n, Mat1& L, Vec1& D, double eps) {
    for (int j = n; j >= 1; --j) {
        for (int i = n; i >= j + 1; --i) {
            double sum = 0.0;
            for (int k = i + 1; k <= n; ++k) {
                sum += L[k][j] * L[k][i];
            }
            L[i][j] = (L[i][j] - sum) / L[i][i];
        }
        double sum = 0.0;
        for (int k = j + 1; k <= n; ++k) {
            sum += L[k][j] * L[k][j];
        }
        const double t = L[j][j] - sum;
        if (t <= L[j][j] * eps) {
            return -1;
        }
        L[j][j] = std::sqrt(t);
    }
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= i - 1; ++j) {
            L[i][j] /= L[i][i];
        }
        D[i] = L[i][i] * L[i][i];
        L[i][i] = 1.0;
    }
    return 0;
}

void ztran(int first, int last, int n, Mat1& L, Vec1& A, Mat1& Zt) {
    for (int i = last; i >= first; --i) {
        for (int j = i + 1; j <= n; ++j) {
            const int mu = static_cast<int>(std::llround(L[j][i]));
            if (mu == 0) continue;
            for (int k = j; k <= n; ++k) {
                L[k][i] -= mu * L[k][j];
            }
            for (int k = 1; k <= n; ++k) {
                Zt[k][j] += mu * Zt[k][i];
            }
            A[i] -= mu * A[j];
        }
    }
}

void src1(int n, Mat1& L, Vec1& D, Vec1& A, Mat1& Zt) {
    int i1 = n - 1;
    bool swapped = true;
    while (swapped) {
        int i = n;
        swapped = false;
        while (!swapped && i > 1) {
            --i;
            if (i <= i1) {
                ztran(i, i, n, L, A, Zt);
            }
            const double lij = L[i + 1][i];
            const double delta = D[i] + lij * lij * D[i + 1];
            if (delta < D[i + 1]) {
                const double eta = D[i] / delta;
                const double mu = D[i + 1] * lij / delta;
                D[i] = eta * D[i + 1];
                D[i + 1] = delta;
                for (int j = 1; j <= i - 1; ++j) {
                    const double t0 = L[i][j];
                    const double t1 = L[i + 1][j];
                    L[i][j] = t1 - lij * t0;
                    L[i + 1][j] = mu * t1 + eta * t0;
                }
                L[i + 1][i] = mu;
                for (int j = i + 2; j <= n; ++j) {
                    std::swap(L[j][i], L[j][i + 1]);
                }
                for (int j = 1; j <= n; ++j) {
                    std::swap(Zt[j][i], Zt[j][i + 1]);
                }
                std::swap(A[i], A[i + 1]);
                i1 = i;
                swapped = true;
            }
        }
    }
}

void invLt(int n, Mat1& L) {
    Vec1 tmp(n + 1, 0.0);
    Mat1 inv = zeros1(n);
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= i - 1; ++j) {
            tmp[j] = L[i][j];
        }
        for (int j = 1; j <= i - 1; ++j) {
            double sum = 0.0;
            for (int k = j; k <= i - 1; ++k) {
                sum += inv[k][j] * tmp[k];
            }
            inv[i][j] = -sum / L[i][i];
        }
        inv[i][i] = 1.0 / L[i][i];
    }
    L = inv;
}

void chiStart(int n, const Vec1& D, const Mat1& L, Vec1& dist, Vec1& E,
              const Vec1& A, Vec1& tm, Vec1& t) {
    const double big = 1e10;
    for (int i = 1; i <= n; ++i) {
        dist[i] = std::llround(A[i]) - A[i];
    }
    for (int i = 1; i <= n; ++i) {
        E[i] = 0.0;
        for (int j = i; j <= n; ++j) {
            E[i] += L[j][i] * dist[j];
        }
    }
    double t0 = 0.0;
    for (int i = 1; i <= n; ++i) {
        t0 += D[i] * E[i] * E[i];
    }
    tm[0] = t0;
    tm[1] = big;

    for (int i = 1; i <= n; ++i) {
        t[i] = t0;
        if (dist[i] < 0.0) {
            for (int j = 1; j <= i; ++j) {
                t[i] += D[j] * L[i][j] * (2.0 * E[j] + L[i][j]);
            }
        } else {
            for (int j = 1; j <= i; ++j) {
                t[i] -= D[j] * L[i][j] * (2.0 * E[j] - L[i][j]);
            }
        }
        if (t[i] < tm[0]) {
            tm[1] = tm[0];
            tm[0] = t[i];
        } else if (t[i] < tm[1]) {
            tm[1] = t[i];
        }
    }
}

void fi71(double chi, int maxCan, int n, const Vec1& A, const Vec1& D, const Mat1& L,
          Vec1& lef, Vec1& left, Vec1& right, Vec1& dist, Vec1& end, Vec1& dq,
          int& ncan, Vec1& disall, Mat1& cands, int& pos) {
    bool ended = false;
    right[n + 1] = chi;
    left[n + 1] = 0.0;
    for (int i = 1; i <= n - 1; ++i) {
        dq[i] = D[i + 1] / D[i];
    }
    dq[n] = 1.0 / D[n];

    ncan = 0;
    int i = n + 1;
    int iold = i;

    while (true) {
        --i;
        if (iold <= i) {
            lef[i] += L[i + 1][i];
        } else {
            lef[i] = dinki(n, L, dist, i);
        }
        iold = i;
        right[i] = (right[i + 1] - left[i + 1]) * dq[i];
        const double reach = std::sqrt(right[i]);
        const double delta = A[i] - reach - lef[i];
        dist[i] = static_cast<double>(jnt2(delta)) - A[i];
        if (dist[i] > (reach - lef[i])) {
            backTrack(n, i, end, dist, lef, left, ended);
        } else {
            end[i] = reach - lef[i] - 1.0;
            left[i] = std::pow(dist[i] + lef[i], 2);
        }
        if (i == 1) {
            double tMax = 0.0;
            int maxPos = 1;
            collectCand(n, maxCan, D[1], lef[1], left[1], right[1], chi, dist, end[1],
                        ncan, disall, cands, tMax, maxPos);
            backTrack(n, i, end, dist, lef, left, ended);
        }
        if (ncan > 10000 || ended) {
            break;
        }
    }

    double dmin = 1e20;
    const int upper = std::min(ncan, maxCan);
    for (int iCan = 1; iCan <= upper; ++iCan) {
        if (disall[iCan] < dmin) {
            dmin = disall[iCan];
            pos = iCan;
        }
        for (int j = 1; j <= n; ++j) {
            cands[j][iCan] += A[j];
        }
    }
}

bool jacobiEigenvaluesSymmetric(const Mat& in, Vec& values) {
    const int n = in.size();
    if (n == 0) return false;
    Mat a = in;
    for (int i = 0; i < n; ++i) {
        if (a[i].size() != n) return false;
        for (int j = 0; j < n; ++j) {
            if (!std::isfinite(a[i][j])) return false;
        }
    }
    const int maxIter = std::max(50, 100 * n * n);
    for (int iter = 0; iter < maxIter; ++iter) {
        int p = 0;
        int q = (n > 1) ? 1 : 0;
        double maxOff = 0.0;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                const double v = std::fabs(a[i][j]);
                if (v > maxOff) {
                    maxOff = v;
                    p = i;
                    q = j;
                }
            }
        }
        if (maxOff < 1e-12 || n == 1) {
            values = Vec(n, 0.0);
            for (int i = 0; i < n; ++i) {
                values[i] = a[i][i];
            }
            std::sort(values.begin(), values.end());
            return true;
        }
        const double app = a[p][p];
        const double aqq = a[q][q];
        const double apq = a[p][q];
        const double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double c = std::cos(phi);
        const double s = std::sin(phi);
        for (int k = 0; k < n; ++k) {
            if (k == p || k == q) continue;
            const double akp = a[k][p];
            const double akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[p][k] = a[k][p];
            a[k][q] = s * akp + c * akq;
            a[q][k] = a[k][q];
        }
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
    }
    values = Vec(n, 0.0);
    for (int i = 0; i < n; ++i) {
        values[i] = a[i][i];
    }
    std::sort(values.begin(), values.end());
    return true;
}

bool lambdaSolve(const Mat& Q, const Vec& a, Vec& aFix, double& ratio, double& boot, LambdaDebug* debug = nullptr) {
    if (debug) {
        *debug = LambdaDebug();
    }
    const int n = a.size();
    if (n == 0) {
        if (debug) debug->failReason = QStringLiteral("empty_float_vector");
        return false;
    }
    const int maxCan = 8;

    Mat1 L = zeros1(n);
    Mat1 Zt = zeros1(n);
    Vec1 A(n + 1, 0.0);
    Vec1 D(n + 1, 0.0);
    Vec1 Ak(n + 1, 0.0);

    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= n; ++j) {
            L[i][j] = Q[i - 1][j - 1];
        }
        Zt[i][i] = 1.0;
        A[i] = a[i - 1];
    }

    for (int i = 1; i <= n; ++i) {
        const double frac = (A[i] >= 0.0) ? (A[i] - std::floor(A[i])) : (A[i] - std::ceil(A[i]));
        Ak[i] = A[i] - frac;
        A[i] = frac;
    }

    if (fmfac(n, L, D, 1e-9) < 0) {
        if (debug) debug->failReason = QStringLiteral("factorization_failed");
        return false;
    }

    src1(n, L, D, A, Zt);

    boot = 1.0;
    for (int i = 1; i <= n; ++i) {
        boot *= bootStrapProb(std::sqrt(D[i]));
    }

    invLt(n, L);
    for (int i = 1; i <= n; ++i) {
        if (D[i] == 0.0) {
            if (debug) debug->failReason = QStringLiteral("zero_decorrelated_variance");
            return false;
        }
        D[i] = 1.0 / D[i];
    }

    Vec1 dist(n + 1, 0.0);
    Vec1 E(n + 1, 0.0);
    Vec1 tm(2, 0.0);
    Vec1 t(n + 1, 0.0);
    chiStart(n, D, L, dist, E, A, tm, t);
    const double chi = tm[1] + 1e-6;
    if (debug) {
        debug->chi = chi;
    }

    Vec1 lef(n + 1, 0.0);
    Vec1 left(n + 2, 0.0);
    Vec1 right(n + 2, 0.0);
    Vec1 end(n + 1, 0.0);
    Vec1 dq(n + 1, 0.0);
    Vec1 disall(maxCan + 1, 0.0);
    Mat1 cands(n + 1, Vec1(maxCan + 1, 0.0));
    int ncan = 0;
    int pos = 1;

    fi71(chi, maxCan, n, A, D, L, lef, left, right, dist, end, dq, ncan, disall, cands, pos);
    if (ncan >= 10000 || ncan < 1) {
        if (debug) {
            debug->nCan = ncan;
            debug->failReason = ncan >= 10000 ? QStringLiteral("too_many_candidates")
                                              : QStringLiteral("no_candidate");
        }
        return false;
    }

    Vec1 tmp(n + 1, 0.0);
    for (int k = 1; k <= maxCan; ++k) {
        for (int i = 1; i <= n; ++i) {
            tmp[i] = 0.0;
            for (int j = 1; j <= n; ++j) {
                tmp[i] += Zt[i][j] * cands[j][k];
            }
        }
        for (int i = 1; i <= n; ++i) {
            cands[i][k] = tmp[i];
        }
    }

    const int kept = std::min(ncan, maxCan);
    QVector<int> order;
    order.reserve(kept);
    for (int k = 1; k <= kept; ++k) {
        order.push_back(k);
    }
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        return disall[lhs] < disall[rhs];
    });
    Vec1 disSorted(maxCan + 1, 0.0);
    Mat1 candSorted(n + 1, Vec1(maxCan + 1, 0.0));
    for (int out = 1; out <= kept; ++out) {
        const int src = order[out - 1];
        disSorted[out] = disall[src];
        for (int i = 1; i <= n; ++i) {
            candSorted[i][out] = cands[i][src];
        }
    }
    disall = disSorted;
    cands = candSorted;

    if (debug) {
        debug->nCan = ncan;
        debug->kept = kept;
        debug->dist1 = disall[1];
        debug->dist2 = disall[2];
        debug->ratioNumerator = disall[2];
        debug->ratioDenominator = disall[1];
        for (int k = 1; k <= debug->kept; ++k) {
            LambdaCandidate cand;
            cand.distSearch = disall[k];
            cand.fixed = Vec(n, 0.0);
            for (int i = 1; i <= n; ++i) {
                cand.fixed[i - 1] = cands[i][k] + Ak[i];
            }
            bool distOk = false;
            cand.distOriginal = quadraticDistance(Q, a, cand.fixed, distOk);
            if (!distOk) {
                cand.distOriginal = std::numeric_limits<double>::quiet_NaN();
            }
            debug->candidates.push_back(cand);
        }
    }

    aFix = Vec(n, 0.0);
    for (int i = 1; i <= n; ++i) {
        A[i] = cands[i][1] + Ak[i];
        aFix[i - 1] = A[i];
    }

    ratio = 0.0;
    if (kept >= 2 && std::isfinite(disall[1]) && std::isfinite(disall[2])) {
        if (disall[1] == 0.0) {
            ratio = (disall[2] == 0.0) ? 0.0 : std::numeric_limits<double>::infinity();
            if (debug) {
                debug->ratioMode = QStringLiteral("great_zero_best_distance");
            }
        } else {
            ratio = disall[2] / disall[1];
            if (debug) {
                debug->ratioMode = QStringLiteral("great_dist2_over_dist1");
            }
        }
        if (debug) {
            debug->ratioNumerator = disall[2];
            debug->ratioDenominator = disall[1];
        }
    } else if (debug) {
        debug->ratioMode = QStringLiteral("insufficient_candidates");
    }
    return true;
}

bool computeNlFloat(const Sat& sat, int wlInt, double ambMeters,
                    double& nlFloat, double& kCoeff, double& cBias) {
    if (sat.f1 <= 0.0 || sat.f2 <= 0.0) return false;
    const double df = sat.f1 - sat.f2;
    if (std::fabs(df) < 1e-6) return false;
    kCoeff = (sat.f1 + sat.f2) / GnssUtils::kC;
    cBias = -static_cast<double>(wlInt) * sat.f2 / df;
    nlFloat = kCoeff * ambMeters + cBias;
    return true;
}

bool melbourneWubbenaCycle(const Sat& sat, double& mw, double& phaseCycles, double& codeCycles) {
    mw = 0.0;
    phaseCycles = 0.0;
    codeCycles = 0.0;
    if (sat.C1 == 0.0 || sat.C2 == 0.0 || sat.L1 == 0.0 || sat.L2 == 0.0 ||
        sat.lambda1 <= 0.0 || sat.lambda2 <= 0.0 ||
        sat.f1 <= 0.0 || sat.f2 <= 0.0) {
        return false;
    }
    const double lambdaWl = GnssUtils::kC / (sat.f1 - sat.f2);
    const double narrowDenom = sat.lambda1 + sat.lambda2;
    if (!std::isfinite(lambdaWl) || std::fabs(lambdaWl) < 1e-9 ||
        std::fabs(narrowDenom) < 1e-9) {
        return false;
    }
    phaseCycles = (sat.L1 + sat.phaseBias1 / sat.lambda1) -
                  (sat.L2 + sat.phaseBias2 / sat.lambda2);
    const double codeNlMeters = (sat.lambda2 * (sat.C1 + sat.codeBias1) +
                                 sat.lambda1 * (sat.C2 + sat.codeBias2)) / narrowDenom;
    codeCycles = codeNlMeters / lambdaWl;
    mw = phaseCycles - codeCycles;
    return std::isfinite(mw);
}

void logQddDiagnostics(QStringList& logLines, const QString& sys, const Mat& Qdd, const QString& tag) {
    const int m = Qdd.size();
    QStringList qDiag;
    double qDiagMin = std::numeric_limits<double>::infinity();
    double qDiagMax = -std::numeric_limits<double>::infinity();
    double qDiagSum = 0.0;
    for (int i = 0; i < m; ++i) {
        qDiag << QString::number(Qdd[i][i], 'g', 12);
        qDiagMin = std::min(qDiagMin, Qdd[i][i]);
        qDiagMax = std::max(qDiagMax, Qdd[i][i]);
        qDiagSum += Qdd[i][i];
    }
    logLines << QString("AR SYS=%1 %2QDD size=%3 diag=%4")
                    .arg(sys)
                    .arg(tag)
                    .arg(m)
                    .arg(qDiag.join(QLatin1Char(',')));
    logLines << QString("AR SYS=%1 %2QDD_SUMMARY min=%3 max=%4 avg=%5")
                    .arg(sys)
                    .arg(tag)
                    .arg(qDiagMin, 0, 'g', 12)
                    .arg(qDiagMax, 0, 'g', 12)
                    .arg(qDiagSum / std::max(1, m), 0, 'g', 12);
    Vec qEig;
    if (jacobiEigenvaluesSymmetric(Qdd, qEig) && !qEig.isEmpty()) {
        QStringList eigParts;
        for (double eig : qEig) {
            eigParts << QString::number(eig, 'g', 12);
        }
        const double minAbsEig = std::fabs(qEig.first());
        const double maxAbsEig = std::fabs(qEig.last());
        const double cond = minAbsEig > 0.0 ? maxAbsEig / minAbsEig
                                            : std::numeric_limits<double>::infinity();
        logLines << QString("AR SYS=%1 %2QDD_EIG values=%3 min=%4 max=%5 cond=%6")
                        .arg(sys)
                        .arg(tag)
                        .arg(eigParts.join(QLatin1Char(',')))
                        .arg(qEig.first(), 0, 'g', 12)
                        .arg(qEig.last(), 0, 'g', 12)
                        .arg(cond, 0, 'g', 12);
    } else {
        logLines << QString("AR SYS=%1 %2QDD_EIG status=fail").arg(sys).arg(tag);
    }
}

AttemptEval evaluateAttempt(const QString& sys,
                            const QVector<int>& keep,
                            const QVector<int>& satIndex,
                            const QVector<QString>& ddNames,
                            const Vec& ddFloatAll,
                            const Mat& QddAll,
                            const QVector<bool>& ddStrongNlHistory,
                            const Options& opt,
                            QStringList& logLines,
                            int round,
                            const QString& kind) {
    AttemptEval out;
    const Vec ddFloat = subVector(ddFloatAll, keep);
    const Mat Qdd = subMatrix(QddAll, keep);
    double qDiagMin = std::numeric_limits<double>::infinity();
    double qDiagMax = -std::numeric_limits<double>::infinity();
    double qDiagSum = 0.0;
    for (int i = 0; i < Qdd.size(); ++i) {
        qDiagMin = std::min(qDiagMin, Qdd[i][i]);
        qDiagMax = std::max(qDiagMax, Qdd[i][i]);
        qDiagSum += Qdd[i][i];
    }
    out.qddMin = qDiagMin;
    out.qddMax = qDiagMax;
    out.qddAvg = qDiagSum / std::max(1, Qdd.size());

    Vec ddFixed;
    LambdaDebug lambdaDebug;
    double ratio = 0.0;
    double boot = 0.0;
    const QString prefix = (round == 0)
                               ? QString()
                               : QString("PAR round=%1 kind=%2 ").arg(round).arg(kind);
    logQddDiagnostics(logLines, sys, Qdd, prefix);
    if (!lambdaSolve(Qdd, ddFloat, ddFixed, ratio, boot, &lambdaDebug)) {
        out.rejectReason = QStringLiteral("lambda_fail");
        out.lambdaDebug = lambdaDebug;
        logLines << QString("AR SYS=%1 PAR_ATTEMPT round=%2 kind=%3 status=skip reason=lambda_fail detail=%4 nCan=%5 kept=%6 chi=%7 keptDD=%8")
                        .arg(sys)
                        .arg(round)
                        .arg(kind)
                        .arg(lambdaDebug.failReason.isEmpty() ? QStringLiteral("-") : lambdaDebug.failReason)
                        .arg(lambdaDebug.nCan)
                        .arg(lambdaDebug.kept)
                        .arg(lambdaDebug.chi, 0, 'g', 12)
                        .arg(joinDds(ddNames, keep));
        return out;
    }
    out.lambdaOk = true;
    out.fixed = ddFixed;
    out.lambdaDebug = lambdaDebug;
    out.ratioRaw = ratio;
    out.ratio = std::min(ratio, 99.0);
    out.boot = boot;

    logLines << QString("AR SYS=%1 PAR_ATTEMPT round=%2 kind=%3 kept=%4 keptDD=%5 ratio=%6 ratioRaw=%7 boot=%8 nCan=%9 candKept=%10 dist1=%11 dist2=%12 ratioMode=%13")
                    .arg(sys)
                    .arg(round)
                    .arg(kind)
                    .arg(keep.size())
                    .arg(joinDds(ddNames, keep))
                    .arg(out.ratio, 0, 'f', 4)
                    .arg(out.ratioRaw, 0, 'g', 12)
                    .arg(out.boot, 0, 'f', 4)
                    .arg(lambdaDebug.nCan)
                    .arg(lambdaDebug.kept)
                    .arg(lambdaDebug.dist1, 0, 'g', 12)
                    .arg(lambdaDebug.dist2, 0, 'g', 12)
                    .arg(lambdaDebug.ratioMode.isEmpty() ? QStringLiteral("-") : lambdaDebug.ratioMode);
    for (int ci = 0; ci < lambdaDebug.candidates.size(); ++ci) {
        const LambdaCandidate& cand = lambdaDebug.candidates[ci];
        QStringList fixedParts;
        QStringList deltaParts;
        for (int i = 0; i < cand.fixed.size(); ++i) {
            fixedParts << QString::number(cand.fixed[i], 'f', 4);
            deltaParts << QString::number(ddFloat[i] - cand.fixed[i], 'g', 12);
        }
        logLines << QString("AR SYS=%1 PAR_CAND round=%2 rank=%3 distSearch=%4 distOriginal=%5 fixed=%6 floatMinusFixed=%7")
                        .arg(sys)
                        .arg(round)
                        .arg(ci + 1)
                        .arg(cand.distSearch, 0, 'g', 12)
                        .arg(cand.distOriginal, 0, 'g', 12)
                        .arg(fixedParts.join(QLatin1Char(',')))
                        .arg(deltaParts.join(QLatin1Char(',')));
    }

    for (int i = 0; i < keep.size(); ++i) {
        const int globalIdx = keep[i];
        const double nlDev = std::fabs(ddFloat[i] - ddFixed[i]);
        if (nlDev > out.maxNlDev || out.worstLocal < 0) {
            out.maxNlDev = nlDev;
            out.worstLocal = i;
            out.worstDd = ddNames[globalIdx];
            out.worstQdd = Qdd[i][i];
        }
        logLines << QString("AR SYS=%1 PAR_DD round=%2 DD=%3 NLfloat=%4 NLfix=%5 NLdev=%6 Qdd=%7")
                        .arg(sys)
                        .arg(round)
                        .arg(ddNames[globalIdx])
                        .arg(ddFloat[i], 0, 'f', 4)
                        .arg(ddFixed[i], 0, 'f', 4)
                        .arg(nlDev, 0, 'f', 4)
                        .arg(Qdd[i][i], 0, 'g', 12);
    }

    double effectiveNlMaxDev = opt.nlMaxDev;
    const int worstGlobalIdx = (out.worstLocal >= 0 && out.worstLocal < keep.size()) ? keep[out.worstLocal] : -1;
    if (worstGlobalIdx >= 0 && worstGlobalIdx < ddStrongNlHistory.size() &&
        ddStrongNlHistory[worstGlobalIdx]) {
        effectiveNlMaxDev *= opt.histRelaxFactor;
    }

    if (out.ratio < opt.nlRatio) {
        out.rejectReason = QStringLiteral("ratio_fail");
    } else if (opt.nlBoot > 0.0 && out.boot < opt.nlBoot) {
        out.rejectReason = QStringLiteral("boot_fail");
    } else if (opt.nlMaxDev > 0.0 && out.maxNlDev > effectiveNlMaxDev) {
        out.rejectReason = QStringLiteral("nl_dev_fail");
    } else {
        out.accepted = true;
        out.rejectReason = QStringLiteral("-");
    }
    logLines << QString("AR SYS=%1 PAR_ATTEMPT_RESULT round=%2 status=%3 reason=%4 ratio=%5 ratioRaw=%6 boot=%7 maxNlDev=%8 nlDevThres=%9 worstDD=%10 worstQdd=%11")
                    .arg(sys)
                    .arg(round)
                    .arg(out.accepted ? QStringLiteral("accept") : QStringLiteral("reject"))
                    .arg(out.rejectReason)
                    .arg(out.ratio, 0, 'f', 4)
                    .arg(out.ratioRaw, 0, 'g', 12)
                    .arg(out.boot, 0, 'f', 4)
                    .arg(out.maxNlDev, 0, 'f', 4)
                    .arg(effectiveNlMaxDev, 0, 'f', 4)
                    .arg(out.worstDd.isEmpty() ? QStringLiteral("-") : out.worstDd)
                    .arg(out.worstQdd, 0, 'g', 12);
    (void)satIndex;
    return out;
}

} // namespace

Result run(const QDateTime& utc, const QVector<Sat>& sats, const Mat& Q, const Options& opt) {
    Result result;
    result.logLines << QString("AR CORE begin utc=%1 inputSats=%2 qSize=%3 opt(minEle=%4 minMw=%5 minPairs=%6 wlDev=%7 wlSig=%8 ratio=%9 boot=%10 nlDev=%11 sigma=%12 glo=%13 histAid=%14 histFix=%15 histLock=%16 warmLock=%17 relax=%18)")
                           .arg(utc.toString(Qt::ISODateWithMs))
                           .arg(sats.size())
                           .arg(Q.size())
                           .arg(opt.minEleDeg, 0, 'f', 2)
                           .arg(opt.minMwSamples)
                           .arg(opt.minFixPairs)
                           .arg(opt.wlMaxDev, 0, 'f', 4)
                           .arg(opt.wlMaxSig, 0, 'f', 4)
                           .arg(opt.nlRatio, 0, 'f', 3)
                           .arg(opt.nlBoot, 0, 'f', 4)
                           .arg(opt.nlMaxDev, 0, 'f', 4)
                           .arg(opt.constraintSigma, 0, 'f', 5)
                           .arg(opt.useGlonass ? "on" : "off")
                           .arg(opt.useHistoryAid ? "on" : "off")
                           .arg(opt.histMinFixEpochs)
                           .arg(opt.histMinLockEpochs)
                           .arg(opt.partialWarmupLockEpochs)
                           .arg(opt.histRelaxFactor, 0, 'f', 3);
    QMap<QChar, QVector<Sat>> sysSats;
    QMap<QString, int> coreSkipCounts;
    auto coreSkip = [&](const Sat& sat, const QString& reason, const QString& detail = QString()) {
        ++coreSkipCounts[reason];
        result.logLines << QString("AR CORE SAT=%1 status=skip reason=%2%3")
                               .arg(sat.prn,
                                    reason,
                                    detail.isEmpty() ? QString() : QString(" %1").arg(detail));
    };
    for (const Sat& sat : sats) {
        if (sat.L1 == 0.0 || sat.L2 == 0.0 || sat.C1 == 0.0 || sat.C2 == 0.0) {
            coreSkip(sat, QStringLiteral("missing_obs"));
            continue;
        }
        if (!sat.hasBias1 || !sat.hasBias2) {
            coreSkip(sat, QStringLiteral("missing_bias"),
                     QString("bias1=%1 bias2=%2").arg(sat.hasBias1 ? "Y" : "N").arg(sat.hasBias2 ? "Y" : "N"));
            continue;
        }
        if (sat.sii1 == 0 || sat.sii2 == 0) {
            coreSkip(sat, QStringLiteral("non_integer_sii"),
                     QString("sii1=%1 sii2=%2").arg(sat.sii1).arg(sat.sii2));
            continue;
        }
        if (sat.sys == QLatin1Char('R') && !opt.useGlonass) {
            coreSkip(sat, QStringLiteral("glonass_disabled"));
            continue;
        }
        if (sat.eleRad * 180.0 / kPi < opt.minEleDeg) {
            coreSkip(sat, QStringLiteral("low_elevation"),
                     QString("ele=%1").arg(sat.eleRad * 180.0 / kPi, 0, 'f', 2));
            continue;
        }
        if (sat.lambda1 <= 0.0 || sat.lambda2 <= 0.0) {
            coreSkip(sat, QStringLiteral("invalid_wavelength"));
            continue;
        }
        sysSats[sat.sys].push_back(sat);
    }
    QStringList coreSkipParts;
    for (auto it = coreSkipCounts.constBegin(); it != coreSkipCounts.constEnd(); ++it) {
        coreSkipParts << QString("%1=%2").arg(it.key()).arg(it.value());
    }
    result.logLines << QString("AR CORE grouped systems=%1 coreSkipped=%2")
                           .arg(sysSats.size())
                           .arg(coreSkipParts.isEmpty() ? QStringLiteral("-") : coreSkipParts.join(QLatin1Char(',')));

    for (auto itSys = sysSats.begin(); itSys != sysSats.end(); ++itSys) {
        const QString sys = QString(itSys.key());
        QVector<Sat> cand = itSys.value();
        result.logLines << QString("AR SYS=%1 status=start candidates=%2").arg(sys).arg(cand.size());
        struct WLSat {
            Sat sat;
            double mwRaw = 0.0;
            double mwPhase = 0.0;
            double mwCode = 0.0;
            double mw = 0.0;
            double mwSig = 0.0;
            int wlInt = 0;
            double nlFloat = 0.0;
            double kCoeff = 0.0;
            double cBias = 0.0;
        };
        QVector<WLSat> preWl;

        for (const Sat& sat : cand) {
            if (sat.wli1 == 0 || sat.wli2 == 0) {
                result.logLines << QString("AR SYS=%1 SAT=%2 status=skip reason=non_integer_wl wli1=%3 wli2=%4")
                                       .arg(sys)
                                       .arg(sat.prn)
                                       .arg(sat.wli1)
                                       .arg(sat.wli2);
                continue;
            }
            if ((sat.ambIndex1 < 0 && sat.ambIndex2 < 0) ||
                (std::fabs(sat.ambCoeff1) < 1e-12 && std::fabs(sat.ambCoeff2) < 1e-12)) {
                result.logLines << QString("AR SYS=%1 SAT=%2 status=skip reason=invalid_amb_coeff idx1=%3 idx2=%4 coeff1=%5 coeff2=%6")
                                       .arg(sys)
                                       .arg(sat.prn)
                                       .arg(sat.ambIndex1)
                                       .arg(sat.ambIndex2)
                                       .arg(sat.ambCoeff1, 0, 'g', 12)
                                       .arg(sat.ambCoeff2, 0, 'g', 12);
                continue;
            }
            double mwRaw = 0.0;
            double mwPhase = 0.0;
            double mwCode = 0.0;
            if (!melbourneWubbenaCycle(sat, mwRaw, mwPhase, mwCode)) {
                result.logLines << QString("AR SYS=%1 SAT=%2 status=skip reason=mw_compute_fail lambda1=%3 lambda2=%4 f1=%5 f2=%6")
                                       .arg(sys)
                                       .arg(sat.prn)
                                       .arg(sat.lambda1, 0, 'g', 12)
                                       .arg(sat.lambda2, 0, 'g', 12)
                                       .arg(sat.f1, 0, 'g', 12)
                                       .arg(sat.f2, 0, 'g', 12);
                continue;
            }
            const double mw = sat.hasMwStat ? sat.mw : mwRaw;
            const double lambdaWl = GnssUtils::kC / (sat.f1 - sat.f2);
            const double codeW1 = sat.lambda2 / (sat.lambda1 + sat.lambda2);
            const double codeW2 = sat.lambda1 / (sat.lambda1 + sat.lambda2);
            double sig = sat.hasMwStat ? sat.mwSig
                                       : std::sqrt(std::pow(opt.sigmaL1 / sat.lambda1, 2) +
                                                   std::pow(opt.sigmaL1 / sat.lambda2, 2) +
                                                   std::pow(codeW1 * opt.sigmaC1 / lambdaWl, 2) +
                                                   std::pow(codeW2 * opt.sigmaC1 / lambdaWl, 2));
            if (sig <= 0.0) sig = 1.0;
            if (opt.minMwSamples > 0) {
                const int nMw = sat.hasMwStat ? sat.mwCount : 1;
                if (nMw < opt.minMwSamples) {
                    result.logLines << QString("AR SYS=%1 SAT=%2 status=skip reason=mw_samples n=%3 min=%4")
                                           .arg(sys)
                                           .arg(sat.prn)
                                           .arg(nMw)
                                           .arg(opt.minMwSamples);
                    continue;
                }
            }
            WLSat entry;
            entry.sat = sat;
            entry.mwRaw = mwRaw;
            entry.mwPhase = mwPhase;
            entry.mwCode = mwCode;
            entry.mw = mw;
            entry.mwSig = sig;
            result.logLines << QString("AR SYS=%1 SAT=%2 status=mw_ready MWraw=%3 MWused=%4 MWsig=%5 MWn=%6 MWphase=%7 MWcode=%8 biasP=%9/%10 biasL=%11/%12")
                                   .arg(sys)
                                   .arg(sat.prn)
                                   .arg(mwRaw, 0, 'f', 4)
                                   .arg(mw, 0, 'f', 4)
                                   .arg(sig, 0, 'f', 4)
                                   .arg(sat.hasMwStat ? sat.mwCount : 1)
                                   .arg(mwPhase, 0, 'f', 4)
                                   .arg(mwCode, 0, 'f', 4)
                                   .arg(sat.codeBias1, 0, 'f', 4)
                                   .arg(sat.codeBias2, 0, 'f', 4)
                                   .arg(sat.phaseBias1, 0, 'f', 4)
                                   .arg(sat.phaseBias2, 0, 'f', 4);
            preWl.push_back(entry);
        }

        if (preWl.size() < 2) {
            result.logLines << QString("AR SYS=%1 status=skip reason=too_few_wl_candidates count=%2")
                                   .arg(sys)
                                   .arg(preWl.size());
            continue;
        }

        int refIndex = -1;
        const QString preferredRef = opt.preferredReferenceBySystem.value(itSys.key());
        if (!preferredRef.isEmpty()) {
            for (int i = 0; i < preWl.size(); ++i) {
                if (preWl[i].sat.prn == preferredRef) {
                    refIndex = i;
                    break;
                }
            }
        }
        QString referenceReason = QStringLiteral("highest_elevation");
        if (refIndex >= 0) {
            referenceReason = QStringLiteral("previous_reference_kept");
        } else {
            refIndex = 0;
            double bestEle = -1e9;
            for (int i = 0; i < preWl.size(); ++i) {
                if (preWl[i].sat.eleRad > bestEle) {
                    bestEle = preWl[i].sat.eleRad;
                    refIndex = i;
                }
            }
        }
        result.logLines << QString("AR SYS=%1 REF=%2 ele=%3 wlCandidates=%4")
                               .arg(sys)
                               .arg(preWl[refIndex].sat.prn)
                               .arg(preWl[refIndex].sat.eleRad * 180.0 / kPi, 0, 'f', 2)
                               .arg(preWl.size());
        result.logLines << QString("AR SYS=%1 REF_SELECTION preferred=%2 selected=%3 reason=%4")
                               .arg(sys)
                               .arg(preferredRef.isEmpty() ? QStringLiteral("-") : preferredRef)
                               .arg(preWl[refIndex].sat.prn)
                               .arg(referenceReason);

        QVector<WLSat> wl;
        wl.push_back(preWl[refIndex]);
        wl[0].wlInt = 0;
        if (!computeNlFloat(wl[0].sat, wl[0].wlInt, wl[0].sat.ambMeters,
                            wl[0].nlFloat, wl[0].kCoeff, wl[0].cBias)) {
            result.logLines << QString("AR SYS=%1 REF=%2 status=skip reason=nl_float_fail")
                                   .arg(sys)
                                   .arg(wl[0].sat.prn);
            continue;
        }
        for (int i = 0; i < preWl.size(); ++i) {
            if (i == refIndex) continue;
            WLSat entry = preWl[i];
            const double ddMw = entry.mw - preWl[refIndex].mw;
            const double ddSig = std::sqrt(entry.mwSig * entry.mwSig +
                                           preWl[refIndex].mwSig * preWl[refIndex].mwSig);
            const double ddDiff = std::fabs(ddMw - std::round(ddMw));
            const bool strongWlHistory = ddHasStrongWlHistory(entry.sat, preWl[refIndex].sat, opt);
            const double wlMaxDev = opt.wlMaxDev * (strongWlHistory ? opt.histRelaxFactor : 1.0);
            const double wlMaxSig = opt.wlMaxSig * (strongWlHistory ? opt.histRelaxFactor : 1.0);
            if (ddDiff > wlMaxDev || ddSig > wlMaxSig) {
                result.logLines << QString("AR SYS=%1 DDWL=%2-%3 status=skip reason=wl_screen dd=%4 diff=%5 sig=%6 mw=%7 refMw=%8 histRelax=%9 devThres=%10 sigThres=%11 lock=%12/%13 wlFix=%14/%15")
                                       .arg(sys)
                                       .arg(entry.sat.prn)
                                       .arg(preWl[refIndex].sat.prn)
                                       .arg(ddMw, 0, 'f', 4)
                                       .arg(ddDiff, 0, 'f', 4)
                                       .arg(ddSig, 0, 'f', 4)
                                       .arg(entry.mw, 0, 'f', 4)
                                       .arg(preWl[refIndex].mw, 0, 'f', 4)
                                       .arg(strongWlHistory ? "yes" : "no")
                                       .arg(wlMaxDev, 0, 'f', 4)
                                       .arg(wlMaxSig, 0, 'f', 4)
                                       .arg(entry.sat.lockEpochs)
                                       .arg(preWl[refIndex].sat.lockEpochs)
                                       .arg(entry.sat.wlFixEpochs)
                                       .arg(preWl[refIndex].sat.wlFixEpochs);
                continue;
            }
            entry.wlInt = static_cast<int>(std::llround(ddMw));
            if (!computeNlFloat(entry.sat, entry.wlInt, entry.sat.ambMeters,
                                entry.nlFloat, entry.kCoeff, entry.cBias)) {
                result.logLines << QString("AR SYS=%1 SAT=%2 status=skip reason=nl_float_fail f1=%3 f2=%4 ambMeters=%5 wlDD=%6")
                                       .arg(sys)
                                       .arg(entry.sat.prn)
                                       .arg(entry.sat.f1, 0, 'g', 12)
                                       .arg(entry.sat.f2, 0, 'g', 12)
                                       .arg(entry.sat.ambMeters, 0, 'f', 6)
                                       .arg(entry.wlInt);
                continue;
            }
            result.logLines << QString("AR SYS=%1 DDWL=%2-%3 status=wl_accept dd=%4 sig=%5 WLdd=%6 NLfloat=%7 k=%8 cBias=%9 ambMeters=%10 histRelax=%11 devThres=%12 sigThres=%13 lock=%14/%15 wlFix=%16/%17")
                                   .arg(sys)
                                   .arg(entry.sat.prn)
                                   .arg(preWl[refIndex].sat.prn)
                                   .arg(ddMw, 0, 'f', 4)
                                   .arg(ddSig, 0, 'f', 4)
                                   .arg(entry.wlInt)
                                   .arg(entry.nlFloat, 0, 'f', 4)
                                   .arg(entry.kCoeff, 0, 'g', 12)
                                   .arg(entry.cBias, 0, 'f', 4)
                                   .arg(entry.sat.ambMeters, 0, 'f', 4)
                                   .arg(strongWlHistory ? "yes" : "no")
                                   .arg(wlMaxDev, 0, 'f', 4)
                                   .arg(wlMaxSig, 0, 'f', 4)
                                   .arg(entry.sat.lockEpochs)
                                   .arg(preWl[refIndex].sat.lockEpochs)
                                   .arg(entry.sat.wlFixEpochs)
                                   .arg(preWl[refIndex].sat.wlFixEpochs);
            wl.push_back(entry);
        }

        if (wl.size() < 2) {
            result.logLines << QString("AR SYS=%1 status=skip reason=too_few_wl_candidates count=%2")
                                   .arg(sys)
                                   .arg(wl.size() - 1);
            continue;
        }

        refIndex = 0;

        Vec ddFloat;
        QVector<int> satIndex;
        QVector<bool> ddStrongWlHistory;
        QVector<bool> ddStrongNlHistory;
        for (int i = 0; i < wl.size(); ++i) {
            if (i == refIndex) continue;
            ddFloat.push_back(wl[i].nlFloat - wl[refIndex].nlFloat);
            satIndex.push_back(i);
            ddStrongWlHistory.push_back(ddHasStrongWlHistory(wl[i].sat, wl[refIndex].sat, opt));
            ddStrongNlHistory.push_back(ddHasStrongNlHistory(wl[i].sat, wl[refIndex].sat, opt));
        }
        if (ddFloat.size() < opt.minFixPairs) {
            result.logLines << QString("AR SYS=%1 status=skip reason=too_few_nl_pairs pairs=%2")
                                   .arg(sys)
                                   .arg(ddFloat.size());
            continue;
        }

        const int n = wl.size();
        Mat Qamb = zeros(n, n);
        for (int i = 0; i < n; ++i) {
            const int i1 = wl[i].sat.ambIndex1;
            const int i2 = wl[i].sat.ambIndex2;
            const double ci1 = wl[i].sat.ambCoeff1;
            const double ci2 = wl[i].sat.ambCoeff2;
            for (int j = 0; j < n; ++j) {
                const int j1 = wl[j].sat.ambIndex1;
                const int j2 = wl[j].sat.ambIndex2;
                const double cj1 = wl[j].sat.ambCoeff1;
                const double cj2 = wl[j].sat.ambCoeff2;
                double qij = 0.0;
                if (i1 >= 0 && j1 >= 0 && i1 < Q.size() && j1 < Q.size()) {
                    qij += ci1 * cj1 * Q[i1][j1];
                }
                if (i1 >= 0 && j2 >= 0 && i1 < Q.size() && j2 < Q.size()) {
                    qij += ci1 * cj2 * Q[i1][j2];
                }
                if (i2 >= 0 && j1 >= 0 && i2 < Q.size() && j1 < Q.size()) {
                    qij += ci2 * cj1 * Q[i2][j1];
                }
                if (i2 >= 0 && j2 >= 0 && i2 < Q.size() && j2 < Q.size()) {
                    qij += ci2 * cj2 * Q[i2][j2];
                }
                Qamb[i][j] = qij;
            }
        }
        Mat Qnl = zeros(n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                Qnl[i][j] = wl[i].kCoeff * wl[j].kCoeff * Qamb[i][j];
            }
        }

        const int m = ddFloat.size();
        Mat Qdd = zeros(m, m);
        for (int i = 0; i < m; ++i) {
            const int si = satIndex[i];
            for (int j = 0; j < m; ++j) {
                const int sj = satIndex[j];
                Qdd[i][j] = Qnl[si][sj] - Qnl[si][refIndex] - Qnl[refIndex][sj] + Qnl[refIndex][refIndex];
            }
        }
        result.logLines << QString("AR SYS=%1 UNITS ddFloat=NL_cycle Qdd=NL_cycle2 ddCount=%2")
                               .arg(sys)
                               .arg(m);
        QVector<QString> ddNames;
        for (int i = 0; i < m; ++i) {
            ddNames.push_back(QString("%1-%2")
                                  .arg(wl[satIndex[i]].sat.prn)
                                  .arg(wl[refIndex].sat.prn));
        }

        QVector<int> keep;
        keep.reserve(m);
        for (int i = 0; i < m; ++i) {
            keep.push_back(i);
        }
        QVector<int> dropped;
        AttemptEval selected;
        bool selectedValid = false;
        int round = 0;
        int warmupDroppedCount = 0;
        result.logLines << QString("AR SYS=%1 PAR begin enabled=%2 fullPairs=%3 minPairs=%4 strategy=GREAT-like_drop_bad_DD realtime_no_fix_epoch_lock")
                               .arg(sys)
                               .arg(opt.enablePartialFix ? "yes" : "no")
                               .arg(m)
                               .arg(opt.minFixPairs);

        while (keep.size() >= opt.minFixPairs) {
            const QString kind = (round == 0 && keep.size() == m) ? QStringLiteral("full")
                                                                  : QStringLiteral("partial");
            AttemptEval eval = evaluateAttempt(sys, keep, satIndex, ddNames, ddFloat, Qdd,
                                               ddStrongNlHistory, opt, result.logLines, round, kind);
            if (!eval.lambdaOk) {
                break;
            }
            if (eval.accepted) {
                selected = eval;
                selectedValid = true;
                break;
            }
            if (!opt.enablePartialFix || keep.size() <= opt.minFixPairs) {
                selected = eval;
                selectedValid = true;
                break;
            }

            int localDrop = -1;
            QString dropMetric = QStringLiteral("qdd_diag");
            if (opt.useHistoryAid && round < 2) {
                double maxDiag = -std::numeric_limits<double>::infinity();
                for (int local = 0; local < keep.size(); ++local) {
                    const int globalIdx = keep[local];
                    const int si = satIndex[globalIdx];
                    if (!ddIsWarmup(wl[si].sat, wl[refIndex].sat, opt)) {
                        continue;
                    }
                    const double diag = Qdd[globalIdx][globalIdx];
                    if (diag > maxDiag) {
                        maxDiag = diag;
                        localDrop = local;
                    }
                }
                if (localDrop >= 0) {
                    dropMetric = QStringLiteral("history_warmup_qdd");
                }
            }
            if (localDrop < 0 && eval.rejectReason == QStringLiteral("nl_dev_fail") && eval.worstLocal >= 0) {
                localDrop = eval.worstLocal;
                dropMetric = QStringLiteral("nl_dev");
            } else if (localDrop < 0) {
                double maxDiag = -std::numeric_limits<double>::infinity();
                for (int local = 0; local < keep.size(); ++local) {
                    const double diag = Qdd[keep[local]][keep[local]];
                    if (diag > maxDiag) {
                        maxDiag = diag;
                        localDrop = local;
                    }
                }
            }
            if (localDrop < 0 || localDrop >= keep.size()) {
                selected = eval;
                selectedValid = true;
                break;
            }
            const int globalDrop = keep[localDrop];
            result.logLines << QString("AR SYS=%1 PAR_DROP round=%2 dd=%3 reason=%4 qdd=%5 lastReject=%6 lastRatio=%7 lastMaxNlDev=%8")
                                   .arg(sys)
                                   .arg(round)
                                   .arg(ddNames[globalDrop])
                                   .arg(dropMetric)
                                   .arg(Qdd[globalDrop][globalDrop], 0, 'g', 12)
                                   .arg(eval.rejectReason)
                                   .arg(eval.ratio, 0, 'f', 4)
                                   .arg(eval.maxNlDev, 0, 'f', 4);
            if (dropMetric == QStringLiteral("history_warmup_qdd")) {
                ++warmupDroppedCount;
            }
            dropped.push_back(globalDrop);
            keep.remove(localDrop);
            ++round;
        }

        if (!selectedValid || !selected.accepted) {
            const QString reason = selectedValid ? selected.rejectReason : QStringLiteral("no_valid_attempt");
            result.logLines << QString("AR SYS=%1 status=reject reason=%2 ratio=%3 ratioRaw=%4 boot=%5 maxNlDev=%6 threshold=%7 minPairs=%8 keptPairs=%9 dropped=%10")
                                   .arg(sys)
                                   .arg(reason)
                                   .arg(selected.ratio, 0, 'f', 4)
                                   .arg(selected.ratioRaw, 0, 'g', 12)
                                   .arg(selected.boot, 0, 'f', 4)
                                   .arg(selected.maxNlDev, 0, 'f', 4)
                                   .arg(opt.nlRatio, 0, 'f', 4)
                                   .arg(opt.minFixPairs)
                                   .arg(keep.size())
                                   .arg(joinDds(ddNames, dropped));
            continue;
        }

        const Vec ddFixed = selected.fixed;
        const double ratio = selected.ratio;
        const double ratioRaw = selected.ratioRaw;
        const double boot = selected.boot;
        const double maxNlDev = selected.maxNlDev;
        const QString worstDd = selected.worstDd.isEmpty() ? QStringLiteral("-") : selected.worstDd;
        const double worstQdd = selected.worstQdd;
        const double qDiagMin = selected.qddMin;
        const double qDiagMax = selected.qddMax;
        const double qDiagAvg = selected.qddAvg;
        QStringList ddSignatureParts;
        result.logLines << QString("AR SYS=%1 QUALITY ratio=%2 ratioRaw=%3 boot=%4 maxNlDev=%5 worstDD=%6 worstQdd=%7 qddMin=%8 qddMax=%9 qddAvg=%10 fixType=%11 keptPairs=%12 droppedPairs=%13")
                               .arg(sys)
                               .arg(ratio, 0, 'f', 4)
                               .arg(ratioRaw, 0, 'g', 12)
                               .arg(boot, 0, 'f', 4)
                               .arg(maxNlDev, 0, 'f', 4)
                               .arg(worstDd)
                               .arg(worstQdd, 0, 'g', 12)
                               .arg(qDiagMin, 0, 'g', 12)
                               .arg(qDiagMax, 0, 'g', 12)
                               .arg(qDiagAvg, 0, 'g', 12)
                               .arg(keep.size() == m ? QStringLiteral("full") : QStringLiteral("partial"))
                               .arg(keep.size())
                               .arg(dropped.size());

        QStringList constraintSignatureParts;
        struct CanonicalNode {
            QString prn;
            double nl = 0.0;
            int wl = 0;
        };
        QVector<CanonicalNode> relationNodes;
        relationNodes.push_back({wl[refIndex].sat.prn, 0.0, 0});
        for (int i = 0; i < keep.size(); ++i) {
            const int globalIdx = keep[i];
            const int si = satIndex[globalIdx];
            Constraint cons;
            cons.sys = itSys.key();
            cons.refPrn = wl[refIndex].sat.prn;
            cons.satPrn = wl[si].sat.prn;
            cons.nlFloat = ddFloat[globalIdx];
            cons.nlFixed = ddFixed[i];
            cons.wlFixed = wl[si].wlInt;
            if (wl[si].sat.ambIndex1 >= 0) {
                cons.coeff[wl[si].sat.ambIndex1] = wl[si].kCoeff * wl[si].sat.ambCoeff1;
            }
            if (wl[si].sat.ambIndex2 >= 0) {
                cons.coeff[wl[si].sat.ambIndex2] = wl[si].kCoeff * wl[si].sat.ambCoeff2;
            }
            if (wl[refIndex].sat.ambIndex1 >= 0) {
                cons.coeff[wl[refIndex].sat.ambIndex1] = -wl[refIndex].kCoeff * wl[refIndex].sat.ambCoeff1;
            }
            if (wl[refIndex].sat.ambIndex2 >= 0) {
                cons.coeff[wl[refIndex].sat.ambIndex2] = -wl[refIndex].kCoeff * wl[refIndex].sat.ambCoeff2;
            }
            cons.z = ddFixed[i] - (wl[si].cBias - wl[refIndex].cBias);
            result.constraints.push_back(cons);
            relationNodes.push_back({cons.satPrn, cons.nlFixed, cons.wlFixed});
            ddSignatureParts << QString("%1-%2:%3:%4")
                                    .arg(wl[si].sat.prn)
                                    .arg(wl[refIndex].sat.prn)
                                    .arg(static_cast<int>(std::llround(ddFixed[i])))
                                    .arg(wl[si].wlInt);
            constraintSignatureParts << QString("%1-%2:NL%3:WL%4:z%5")
                                            .arg(cons.satPrn)
                                            .arg(cons.refPrn)
                                            .arg(static_cast<int>(std::llround(cons.nlFixed)))
                                            .arg(cons.wlFixed)
                                            .arg(cons.z, 0, 'f', 6);
            QStringList coeffParts;
            for (auto it = cons.coeff.constBegin(); it != cons.coeff.constEnd(); ++it) {
                coeffParts << QString("%1:%2").arg(it.key()).arg(it.value(), 0, 'g', 12);
            }
            result.logLines << QString("AR SYS=%1 CONSTRAINT=%2 ref=%3 sat=%4 z=%5 coeff=%6")
                                   .arg(sys)
                                   .arg(i)
                                   .arg(wl[refIndex].sat.prn)
                                   .arg(wl[si].sat.prn)
                                   .arg(cons.z, 0, 'f', 6)
                                   .arg(coeffParts.join(QLatin1Char(',')));
        }

        std::sort(ddSignatureParts.begin(), ddSignatureParts.end());
        std::sort(constraintSignatureParts.begin(), constraintSignatureParts.end());
        std::sort(relationNodes.begin(), relationNodes.end(), [](const CanonicalNode& a, const CanonicalNode& b) {
            return a.prn < b.prn;
        });
        const CanonicalNode& canonical = relationNodes.front();
        QStringList canonicalRelationParts;
        for (int i = 1; i < relationNodes.size(); ++i) {
            const CanonicalNode& node = relationNodes[i];
            canonicalRelationParts << QString("%1-%2:NL%3:WL%4")
                                          .arg(node.prn)
                                          .arg(canonical.prn)
                                          .arg(static_cast<int>(std::llround(node.nl - canonical.nl)))
                                          .arg(node.wl - canonical.wl);
        }
        std::sort(canonicalRelationParts.begin(), canonicalRelationParts.end());
        FixedSet fixedSet;
        fixedSet.sys = itSys.key();
        fixedSet.refPrn = wl[refIndex].sat.prn;
        fixedSet.signature = QString("%1|relations=%2")
                                 .arg(sys)
                                 .arg(canonicalRelationParts.join(QLatin1Char(';')));
        fixedSet.wlCount = wl.size();
        fixedSet.nlCount = keep.size();
        fixedSet.ratio = ratio;
        fixedSet.ratioRaw = ratioRaw;
        fixedSet.boot = boot;
        fixedSet.maxNlDev = maxNlDev;
        fixedSet.qddMin = qDiagMin;
        fixedSet.qddMax = qDiagMax;
        fixedSet.qddAvg = qDiagAvg;
        fixedSet.worstDd = worstDd;
        fixedSet.worstNlDev = maxNlDev;
        fixedSet.partial = keep.size() != m;
        fixedSet.warmupDroppedCount = warmupDroppedCount;
        for (int idx : keep) {
            if (idx >= 0 && idx < ddStrongWlHistory.size() && ddStrongWlHistory[idx]) {
                ++fixedSet.histQualifiedWlDdCount;
            }
            if (idx >= 0 && idx < ddStrongNlHistory.size() && ddStrongNlHistory[idx]) {
                ++fixedSet.histQualifiedNlDdCount;
            }
        }
        for (int idx : keep) {
            fixedSet.fixedDds << ddNames[idx];
            fixedSet.fixedPrns << wl[satIndex[idx]].sat.prn;
        }
        for (int idx : dropped) {
            fixedSet.droppedDds << ddNames[idx];
        }
        fixedSet.fixedPrns << fixedSet.refPrn;
        std::sort(fixedSet.fixedPrns.begin(), fixedSet.fixedPrns.end());
        fixedSet.fixedPrns.erase(std::unique(fixedSet.fixedPrns.begin(), fixedSet.fixedPrns.end()),
                                 fixedSet.fixedPrns.end());
        result.fixedSets.push_back(fixedSet);

        if (!result.summary.isEmpty()) {
            result.summary += " ";
        }
        result.summary += QString("AR%1:WL=%2 NL=%3 ratio=%4")
                              .arg(sys)
                              .arg(wl.size())
                              .arg(keep.size())
                              .arg(ratio, 0, 'f', 2);
        result.logLines << QString("AR SYS=%1 status=fixed type=%2 WL=%3 NL=%4 fullNL=%5 ratio=%6 ratioRaw=%7 boot=%8 maxNlDev=%9 histWlQualified=%10 histNlQualified=%11 warmupDropped=%12 fixedDD=%13 droppedDD=%14")
                               .arg(sys)
                               .arg(keep.size() == m ? QStringLiteral("full") : QStringLiteral("partial"))
                               .arg(wl.size())
                               .arg(keep.size())
                               .arg(m)
                               .arg(ratio, 0, 'f', 4)
                               .arg(ratioRaw, 0, 'g', 12)
                               .arg(boot, 0, 'f', 4)
                               .arg(maxNlDev, 0, 'f', 4)
                               .arg(fixedSet.histQualifiedWlDdCount)
                               .arg(fixedSet.histQualifiedNlDdCount)
                               .arg(fixedSet.warmupDroppedCount)
                               .arg(fixedSet.fixedDds.isEmpty() ? QStringLiteral("-") : fixedSet.fixedDds.join(QLatin1Char(',')))
                               .arg(fixedSet.droppedDds.isEmpty() ? QStringLiteral("-") : fixedSet.droppedDds.join(QLatin1Char(',')));
    }

    return result;
}

} // namespace AmbFix
