// Implements BIE candidate search, ambiguity screening, and posterior constraints.
#include "ambbie.h"

#include <QMap>
#include <algorithm>
#include <cmath>
#include <limits>

namespace BieAlgorithm {

namespace {

Mat zeros(int rows, int cols) {
    return Mat(rows, Vec(cols, 0.0));
}

Mat identity(int n) {
    Mat out = zeros(n, n);
    for (int i = 0; i < n; ++i) out[i][i] = 1.0;
    return out;
}

Mat transpose(const Mat& a) {
    if (a.isEmpty()) return Mat();
    Mat out = zeros(a[0].size(), a.size());
    for (int i = 0; i < a.size(); ++i) {
        for (int j = 0; j < a[i].size(); ++j) out[j][i] = a[i][j];
    }
    return out;
}

Mat multiply(const Mat& a, const Mat& b) {
    if (a.isEmpty() || b.isEmpty() || a[0].size() != b.size()) return Mat();
    Mat out = zeros(a.size(), b[0].size());
    for (int i = 0; i < a.size(); ++i) {
        for (int k = 0; k < b.size(); ++k) {
            if (a[i][k] == 0.0) continue;
            for (int j = 0; j < b[0].size(); ++j) out[i][j] += a[i][k] * b[k][j];
        }
    }
    return out;
}

Vec multiply(const Mat& a, const Vec& v) {
    if (a.isEmpty() || a[0].size() != v.size()) return Vec();
    Vec out(a.size(), 0.0);
    for (int i = 0; i < a.size(); ++i) {
        for (int j = 0; j < v.size(); ++j) out[i] += a[i][j] * v[j];
    }
    return out;
}

bool invert(const Mat& input, Mat& inverse) {
    const int n = input.size();
    if (n == 0 || input[0].size() != n) return false;
    Mat aug = zeros(n, n * 2);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) aug[i][j] = input[i][j];
        aug[i][n + i] = 1.0;
    }
    for (int i = 0; i < n; ++i) {
        int pivot = i;
        double maxValue = std::fabs(aug[i][i]);
        for (int r = i + 1; r < n; ++r) {
            if (std::fabs(aug[r][i]) > maxValue) {
                maxValue = std::fabs(aug[r][i]);
                pivot = r;
            }
        }
        if (!std::isfinite(maxValue) || maxValue < 1.0e-14) return false;
        if (pivot != i) std::swap(aug[pivot], aug[i]);
        const double diagonal = aug[i][i];
        for (double& value : aug[i]) value /= diagonal;
        for (int r = 0; r < n; ++r) {
            if (r == i) continue;
            const double factor = aug[r][i];
            if (factor == 0.0) continue;
            for (int c = 0; c < n * 2; ++c) aug[r][c] -= factor * aug[i][c];
        }
    }
    inverse = zeros(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) inverse[i][j] = aug[i][n + j];
    }
    return true;
}

bool choleskyDecompose(const Mat& input, Mat& lower) {
    const int n = input.size();
    if (n == 0) return false;
    for (const Vec& row : input) {
        if (row.size() != n) return false;
    }

    lower = zeros(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = input[i][j];
            for (int k = 0; k < j; ++k) sum -= lower[i][k] * lower[j][k];
            if (!std::isfinite(sum)) return false;
            if (i == j) {
                if (sum <= 1.0e-12) return false;
                lower[i][j] = std::sqrt(sum);
            } else {
                if (std::fabs(lower[j][j]) < 1.0e-12) return false;
                lower[i][j] = sum / lower[j][j];
            }
            if (!std::isfinite(lower[i][j])) return false;
        }
    }
    return true;
}

bool solveUpperTriangular(const Mat& upper, const Vec& rhs, Vec& solution) {
    const int n = upper.size();
    if (n == 0 || rhs.size() != n) return false;
    for (const Vec& row : upper) {
        if (row.size() != n) return false;
    }

    solution = Vec(n, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        double value = rhs[i];
        for (int j = i + 1; j < n; ++j) value -= upper[i][j] * solution[j];
        if (!std::isfinite(value) || std::fabs(upper[i][i]) < 1.0e-12) return false;
        solution[i] = value / upper[i][i];
        if (!std::isfinite(solution[i])) return false;
    }
    return true;
}

bool invertUpperTriangular(const Mat& upper, Mat& inverse) {
    const int n = upper.size();
    if (n == 0) return false;
    inverse = zeros(n, n);
    for (int col = 0; col < n; ++col) {
        Vec rhs(n, 0.0);
        rhs[col] = 1.0;
        Vec solution;
        if (!solveUpperTriangular(upper, rhs, solution)) return false;
        for (int row = 0; row < n; ++row) inverse[row][col] = solution[row];
    }
    return true;
}

bool qrzUpper(Mat work, Mat& upper) {
    const int rows = work.size();
    if (rows == 0) return false;
    const int cols = work[0].size();
    for (const Vec& row : work) {
        if (row.size() != cols) return false;
    }

    upper = zeros(cols, cols);
    for (int col = 0; col < cols; ++col) {
        for (int offset = 0; offset < cols - col; ++offset) {
            double sum = 0.0;
            for (int row = 0; row < rows; ++row) {
                sum += work[row][col] * work[row][col + offset];
            }
            if (!std::isfinite(sum)) return false;
            upper[col][col + offset] += sum;
        }

        double diagonalSquare = upper[col][col];
        if (diagonalSquare < 0.0 && std::fabs(diagonalSquare) < 1.0e-10) {
            diagonalSquare = 0.0;
        }
        if (!std::isfinite(diagonalSquare) || diagonalSquare < 0.0) return false;
        const double diagonal = std::sqrt(diagonalSquare);
        upper[col][col] = diagonal;
        if (diagonal == 0.0) {
            for (int j = col + 1; j < cols; ++j) upper[col][j] = 0.0;
            for (int row = 0; row < rows; ++row) work[row][col] = 0.0;
            continue;
        }

        for (int j = col + 1; j < cols; ++j) upper[col][j] /= diagonal;
        for (int row = 0; row < rows; ++row) {
            const double normalized = work[row][col] / diagonal;
            work[row][col] = normalized;
            for (int j = col + 1; j < cols; ++j) {
                work[row][j] -= upper[col][j] * normalized;
            }
        }
    }
    return true;
}

double nint(double value) {
    return value < 0.0 ? -std::floor(std::fabs(value) + 0.5) : std::floor(value + 0.5);
}

bool isFixable(double fraction, double sigma, const Options& options) {
    if (options.maxFrac <= 0.0 || options.maxSigma <= 0.0) return true;
    return fraction <= options.maxFrac && sigma <= options.maxSigma;
}

bool ldlDecompose(const Mat& q, Mat& l, Vec& d) {
    const int n = q.size();
    if (n == 0 || q[0].size() != n) return false;
    l = zeros(n, n);
    d = Vec(n, 0.0);
    Mat work = q;
    for (int i = n - 1; i >= 0; --i) {
        d[i] = work[i][i];
        if (!std::isfinite(d[i]) || d[i] <= 0.0) return false;
        const double root = std::sqrt(d[i]);
        for (int j = 0; j <= i; ++j) l[i][j] = work[i][j] / root;
        if (std::fabs(l[i][i]) < 1.0e-14) return false;
        for (int j = 0; j < i; ++j) {
            for (int k = 0; k <= j; ++k) work[j][k] -= l[i][k] * l[i][j];
        }
        for (int j = 0; j <= i; ++j) l[i][j] /= l[i][i];
    }
    return true;
}

bool invertLowerTriangular(const Mat& lower, Mat& inverse) {
    const int n = lower.size();
    if (n == 0 || lower[0].size() != n) return false;
    inverse = zeros(n, n);
    for (int col = 0; col < n; ++col) {
        for (int row = col; row < n; ++row) {
            const double diagonal = lower[row][row];
            if (!std::isfinite(diagonal) || std::fabs(diagonal) < 1.0e-14) return false;
            double value = row == col ? 1.0 : 0.0;
            for (int k = col; k < row; ++k) value -= lower[row][k] * inverse[k][col];
            inverse[row][col] = value / diagonal;
        }
    }
    return true;
}

bool decorrel(const Mat& q, const Vec& a, Mat& z, Mat& l, Vec& d, Vec& zFloat, Mat& inverseZt) {
    if (!ldlDecompose(q, l, d)) return false;
    const int n = q.size();
    inverseZt = identity(n);
    // Legacy uses columns 1..n; n-2 is the highest left column in zero-based indexing.
    int i1 = n - 2;
    bool changed = true;
    while (changed) {
        changed = false;
        int i = n - 1;
        while (!changed && i > 0) {
            --i;
            if (i <= i1) {
                for (int j = i + 1; j < n; ++j) {
                    const double mu = nint(l[j][i]);
                    if (mu == 0.0) continue;
                    for (int k = j; k < n; ++k) l[k][i] -= mu * l[k][j];
                    for (int k = 0; k < n; ++k) inverseZt[k][j] += mu * inverseZt[k][i];
                }
            }
            const double delta = d[i] + l[i + 1][i] * l[i + 1][i] * d[i + 1];
            if (delta < d[i + 1]) {
                const double lambda = d[i + 1] * l[i + 1][i] / delta;
                const double eta = d[i] / delta;
                d[i] = eta * d[i + 1];
                d[i + 1] = delta;
                for (int k = 0; k < i; ++k) {
                    const double oldI = l[i][k];
                    const double oldIp = l[i + 1][k];
                    l[i][k] = -l[i + 1][i] * oldI + oldIp;
                    l[i + 1][k] = eta * oldI + lambda * oldIp;
                }
                l[i + 1][i] = lambda;
                for (int k = i + 2; k < n; ++k) std::swap(l[k][i], l[k][i + 1]);
                for (int k = 0; k < n; ++k) std::swap(inverseZt[k][i], inverseZt[k][i + 1]);
                i1 = i;
                changed = true;
            }
        }
    }
    Mat inverseOfInverseZt;
    if (!invert(inverseZt, inverseOfInverseZt)) return false;
    z = transpose(inverseOfInverseZt);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) z[i][j] = nint(z[i][j]);
    }
    zFloat = multiply(transpose(z), a);
    return true;
}

struct SearchResult {
    Mat candidates;
    Vec norms;
};

struct BieSearchDiagnostics {
    double firstNorm = std::numeric_limits<double>::quiet_NaN();
    double secondNorm = std::numeric_limits<double>::quiet_NaN();
    double secondWeight = 0.0;
    double effectiveCandidates = 0.0;
};

SearchResult searchCandidates(const Vec& zFloat, const Mat& l, const Vec& d, int requested) {
    SearchResult out;
    const int n = zFloat.size();
    if (n == 0 || requested <= 0) return out;
    const double inf = std::numeric_limits<double>::max();
    double chi2 = inf;
    Vec distance(n, 0.0), conditional(n, 0.0), integer(n, 0.0), step(n, 0.0);
    Mat ss = zeros(n, n);
    conditional[n - 1] = zFloat[n - 1];
    integer[n - 1] = nint(conditional[n - 1]);
    double left = conditional[n - 1] - integer[n - 1];
    step[n - 1] = left == 0.0 ? 1.0 : (left < 0.0 ? -1.0 : 1.0);
    int maxIndex = requested - 1;
    int k = n - 1;
    while (true) {
        const double newDistance = distance[k] + left * left / d[k];
        if (newDistance < chi2) {
            if (k != 0) {
                --k;
                distance[k] = newDistance;
                for (int j = 0; j <= k; ++j) {
                    ss[k][j] = ss[k + 1][j] + (integer[k + 1] - conditional[k + 1]) * l[k + 1][j];
                }
                conditional[k] = zFloat[k] + ss[k][k];
                integer[k] = nint(conditional[k]);
                left = conditional[k] - integer[k];
                step[k] = left == 0.0 ? 1.0 : (left < 0.0 ? -1.0 : 1.0);
            } else {
                if (out.norms.size() < requested) {
                    out.norms.push_back(newDistance);
                    out.candidates.push_back(integer);
                    if (out.norms.size() == requested) {
                        maxIndex = 0;
                        for (int i = 1; i < out.norms.size(); ++i) {
                            if (out.norms[i] > out.norms[maxIndex]) maxIndex = i;
                        }
                        chi2 = out.norms[maxIndex];
                    }
                } else if (newDistance < chi2) {
                    out.norms[maxIndex] = newDistance;
                    out.candidates[maxIndex] = integer;
                    maxIndex = 0;
                    for (int i = 1; i < out.norms.size(); ++i) {
                        if (out.norms[i] > out.norms[maxIndex]) maxIndex = i;
                    }
                    chi2 = out.norms[maxIndex];
                }
                integer[0] += step[0];
                left = conditional[0] - integer[0];
                step[0] = -step[0] - (step[0] < 0.0 ? -1.0 : (step[0] > 0.0 ? 1.0 : 0.0));
            }
        } else if (k == n - 1) {
            break;
        } else {
            ++k;
            integer[k] += step[k];
            left = conditional[k] - integer[k];
            step[k] = -step[k] - (step[k] < 0.0 ? -1.0 : (step[k] > 0.0 ? 1.0 : 0.0));
        }
    }
    QVector<int> order(out.norms.size());
    for (int i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) { return out.norms[a] < out.norms[b]; });
    Mat sortedCandidates;
    Vec sortedNorms;
    for (int index : order) {
        sortedCandidates.push_back(out.candidates[index]);
        sortedNorms.push_back(out.norms[index]);
    }
    out.candidates = sortedCandidates;
    out.norms = sortedNorms;
    return out;
}

bool bieSearch(const Vec& input, const Mat& covariance, int candidateCount,
               Vec& bie, Mat& bieCovariance, int& actualCandidates,
               BieSearchDiagnostics& diagnostics) {
    const int n = input.size();
    if (n == 0 || covariance.size() != n || candidateCount <= 0) return false;
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(input[i]) || covariance[i].size() != n) return false;
        for (double value : covariance[i]) {
            if (!std::isfinite(value)) return false;
        }
    }
    Vec increments(n, 0.0);
    Vec fractional = input;
    for (int i = 0; i < n; ++i) {
        increments[i] = nint(fractional[i]);
        fractional[i] -= increments[i];
    }
    Mat z, l, inverseZt;
    Vec d, zFloat;
    if (!decorrel(covariance, fractional, z, l, d, zFloat, inverseZt)) return false;
    SearchResult search = searchCandidates(zFloat, l, d, candidateCount);
    actualCandidates = search.candidates.size();
    if (actualCandidates == 0) return false;

    Mat inverseL;
    if (!invertLowerTriangular(l, inverseL)) return false;
    Mat inverseD = zeros(n, n);
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(d[i]) || d[i] <= 0.0) return false;
        inverseD[i][i] = 1.0 / d[i];
    }
    // Match reference implementation: Qz^-1 = L^-1 * D^-1 * L^-T.
    const Mat zCovInverse = multiply(multiply(inverseL, inverseD), transpose(inverseL));
    if (zCovInverse.size() != n) return false;
    Vec weights(actualCandidates, 0.0);
    double weightSum = 0.0;
    double firstNorm = 0.0;
    for (int c = 0; c < actualCandidates; ++c) {
        Vec delta(n, 0.0);
        Vec zCandidate = search.candidates[c];
        for (int i = 0; i < n; ++i) delta[i] = zFloat[i] - zCandidate[i];
        const Vec weightedDelta = multiply(zCovInverse, delta);
        double norm = 0.0;
        for (int i = 0; i < n; ++i) norm += delta[i] * weightedDelta[i];
        if (c == 0) {
            firstNorm = norm;
            diagnostics.firstNorm = norm;
        } else if (c == 1) {
            diagnostics.secondNorm = norm;
        }
        weights[c] = std::exp(-0.5 * (norm - firstNorm));
        weightSum += weights[c];
    }
    if (!std::isfinite(weightSum) || weightSum <= 0.0) return false;
    for (double& weight : weights) weight /= weightSum;
    if (actualCandidates > 1) diagnostics.secondWeight = weights[1];
    double sumSquaredWeights = 0.0;
    for (double weight : weights) sumSquaredWeights += weight * weight;
    if (sumSquaredWeights > 0.0) diagnostics.effectiveCandidates = 1.0 / sumSquaredWeights;
    Vec zBie(n, 0.0);
    for (int c = 0; c < actualCandidates; ++c) {
        for (int i = 0; i < n; ++i) zBie[i] += weights[c] * search.candidates[c][i];
    }
    bie = multiply(inverseZt, zBie);
    if (bie.size() != n) return false;
    for (int i = 0; i < n; ++i) bie[i] += increments[i];
    Mat covZ = zeros(n, n);
    for (int i = 0; i < n; ++i) {
        double variance = 0.0;
        for (int c = 0; c < actualCandidates; ++c) {
            const double diff = zBie[i] - search.candidates[c][i];
            variance += weights[c] * diff * diff;
        }
        covZ[i][i] = std::max(variance, 1.0e-12);
    }
    // Z^-1 = (Z^-T)^T, and inverseZt is Z^-T from decorrelation.
    const Mat inverseZ = transpose(inverseZt);
    bieCovariance = multiply(multiply(inverseZt, covZ), inverseZ);
    return true;
}

bool conditionReferences(Vec& values, Mat& covariance, const QVector<int>& references, double sigma) {
    const int n = values.size();
    const int m = references.size();
    if (n == 0 || m == 0 || covariance.size() != n || sigma <= 0.0) return false;

    for (const Vec& row : covariance) {
        if (row.size() != n) return false;
    }

    Mat h = zeros(m, n);
    Vec target(m, 0.0);
    for (int row = 0; row < m; ++row) {
        const int ref = references[row];
        if (ref < 0 || ref >= n) return false;
        h[row][ref] = 1.0;
        target[row] = nint(values[ref]);
    }

    Mat covarianceRootLower;
    if (!choleskyDecompose(covariance, covarianceRootLower)) return false;
    const Mat covarianceRootUpper = transpose(covarianceRootLower);
    const Mat rootTimesHt = multiply(covarianceRootUpper, transpose(h));

    const int combinedSize = m + n;
    Mat squareRootFilter = zeros(combinedSize, combinedSize);
    for (int i = 0; i < m; ++i) squareRootFilter[i][i] = sigma;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            squareRootFilter[m + i][j] = rootTimesHt[i][j];
        }
        for (int j = 0; j < n; ++j) {
            squareRootFilter[m + i][m + j] = covarianceRootUpper[i][j];
        }
    }

    Mat transformed;
    if (!qrzUpper(squareRootFilter, transformed)) return false;
    Mat innovationRoot = zeros(m, m);
    Mat crossRoot = zeros(m, n);
    Mat posteriorRoot = zeros(n, n);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) innovationRoot[i][j] = transformed[i][j];
        for (int j = 0; j < n; ++j) crossRoot[i][j] = transformed[i][m + j];
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) posteriorRoot[i][j] = transformed[m + i][m + j];
    }

    Mat inverseInnovationRoot;
    if (!invertUpperTriangular(innovationRoot, inverseInnovationRoot)) return false;
    const Mat gainTranspose = multiply(inverseInnovationRoot, crossRoot);
    if (gainTranspose.size() != m) return false;

    Vec innovation(m, 0.0);
    for (int i = 0; i < m; ++i) innovation[i] = target[i] - values[references[i]];
    const Vec correction = multiply(transpose(gainTranspose), innovation);
    if (correction.size() != n) return false;
    for (int i = 0; i < n; ++i) values[i] += correction[i];

    covariance = multiply(transpose(posteriorRoot), posteriorRoot);
    if (covariance.size() != n) return false;
    return true;
}

QString groupKey(const Ambiguity& amb) {
    return QString("%1|%2").arg(amb.system).arg(amb.group);
}

} // namespace

bool search(const Vec& ambiguities, const Mat& covariance,
            Vec& bie, Mat& bieCovariance, int candidateCount) {
    int actualCandidates = 0;
    BieSearchDiagnostics diagnostics;
    return bieSearch(ambiguities, covariance, candidateCount, bie, bieCovariance,
                     actualCandidates, diagnostics);
}

Result run(const QVector<Ambiguity>& ambiguities, const Mat& covariance, const Options& options) {
    Result result;
    if (ambiguities.isEmpty() || covariance.size() != ambiguities.size()) {
        result.logLines << QStringLiteral("BIE status=skip reason=invalid_input");
        return result;
    }
    for (int i = 0; i < covariance.size(); ++i) {
        if (covariance[i].size() != ambiguities.size() || !std::isfinite(ambiguities[i].value)) {
            result.logLines << QStringLiteral("BIE status=skip reason=invalid_input");
            return result;
        }
        for (double value : covariance[i]) {
            if (!std::isfinite(value)) {
                result.logLines << QStringLiteral("BIE status=skip reason=invalid_input");
                return result;
            }
        }
    }

    QMap<QString, QVector<int>> groups;
    for (int i = 0; i < ambiguities.size(); ++i) {
        const Ambiguity& amb = ambiguities[i];
        if (amb.stateIndex < 0 || amb.numEpochs < options.minNumEpochs ||
            amb.elevationDeg < options.minElevationDeg || amb.group.isEmpty()) {
            continue;
        }
        groups[groupKey(amb)].push_back(i);
    }
    QVector<int> activeIndices;
    QMap<QString, QVector<int>> activeGroups;
    bool hasEnoughAmbiguities = false;
    for (auto groupIt = groups.constBegin(); groupIt != groups.constEnd(); ++groupIt) {
        if (groupIt.value().size() < 2) continue;
        if (groupIt.value().size() >= options.minNumSat) hasEnoughAmbiguities = true;
        for (int originalIndex : groupIt.value()) {
            activeGroups[groupIt.key()].push_back(activeIndices.size());
            activeIndices.push_back(originalIndex);
        }
    }
    if (!hasEnoughAmbiguities || activeIndices.isEmpty()) {
        result.logLines << QStringLiteral("BIE status=skip reason=not_enough_resolvable_ambiguities");
        return result;
    }

    Vec values(activeIndices.size(), 0.0);
    Mat q = zeros(activeIndices.size(), activeIndices.size());
    for (int i = 0; i < activeIndices.size(); ++i) {
        values[i] = ambiguities[activeIndices[i]].value;
        for (int j = 0; j < activeIndices.size(); ++j) {
            q[i][j] = covariance[activeIndices[i]][activeIndices[j]];
        }
    }

    QMap<QString, int> referenceByGroup;
    bool anyEnoughGroup = false;
    for (auto groupIt = activeGroups.constBegin(); groupIt != activeGroups.constEnd(); ++groupIt) {
        const QVector<int>& localIndices = groupIt.value();
        int refLocal = -1;
        const QString preferredRef = options.preferredReferenceByGroup.value(groupIt.key());
        if (!preferredRef.isEmpty()) {
            for (int candidate : localIndices) {
                if (ambiguities[activeIndices[candidate]].prn == preferredRef) {
                    refLocal = candidate;
                    break;
                }
            }
        }
        if (refLocal < 0) {
            refLocal = localIndices.first();
            double bestVarianceSum = std::numeric_limits<double>::infinity();
            for (int candidate : localIndices) {
                double varianceSum = 0.0;
                for (int other : localIndices) {
                    varianceSum += q[candidate][candidate] - 2.0 * q[candidate][other] + q[other][other];
                }
                if (varianceSum < bestVarianceSum) {
                    bestVarianceSum = varianceSum;
                    refLocal = candidate;
                }
            }
        }
        referenceByGroup[groupIt.key()] = refLocal;
        result.logLines << QString("BIE group=%1 reference_selection preferred=%2 selected=%3 reason=%4")
                               .arg(groupIt.key())
                               .arg(preferredRef.isEmpty() ? QStringLiteral("-") : preferredRef)
                               .arg(ambiguities[activeIndices[refLocal]].prn)
                               .arg(preferredRef == ambiguities[activeIndices[refLocal]].prn
                                        ? QStringLiteral("previous_reference_kept")
                                        : QStringLiteral("minimum_reference_variance"));
    }
    QVector<int> references;
    references.reserve(referenceByGroup.size());
    for (auto refIt = referenceByGroup.constBegin(); refIt != referenceByGroup.constEnd(); ++refIt) {
        references.push_back(refIt.value());
    }
    if (!conditionReferences(values, q, references, options.constraintSigma)) {
        result.logLines << QStringLiteral("BIE status=skip reason=reference_condition_failed");
        return result;
    }

    Vec bie;
    Mat bieCovariance;
    int actualCandidates = 0;
    BieSearchDiagnostics searchDiagnostics;
    if (!bieSearch(values, q, options.candidateCount, bie, bieCovariance, actualCandidates,
                   searchDiagnostics)) {
        result.logLines << QStringLiteral("BIE status=skip reason=joint_search_failed");
        return result;
    }
    result.candidateCount = actualCandidates;
    result.effectiveCandidates = searchDiagnostics.effectiveCandidates;
    result.logLines << QString("BIE search status=ok ambiguities=%1 groups=%2 candidates=%3 norm1=%4 norm2=%5 weight2=%6 effectiveCandidates=%7")
                           .arg(activeIndices.size())
                           .arg(activeGroups.size())
                           .arg(actualCandidates)
                           .arg(searchDiagnostics.firstNorm, 0, 'g', 12)
                           .arg(searchDiagnostics.secondNorm, 0, 'g', 12)
                           .arg(searchDiagnostics.secondWeight, 0, 'g', 12)
                           .arg(searchDiagnostics.effectiveCandidates, 0, 'g', 12);

    for (auto groupIt = activeGroups.constBegin(); groupIt != activeGroups.constEnd(); ++groupIt) {
        const QVector<int>& localIndices = groupIt.value();
        const int refLocal = referenceByGroup.value(groupIt.key(), -1);
        int fixedCount = 0;
        double maxFraction = 0.0;
        double maxSigma = 0.0;
        for (int local : localIndices) {
            const double fraction = std::fabs(bie[local] - nint(bie[local]));
            const double sigma = std::sqrt(std::max(0.0, bieCovariance[local][local]));
            maxFraction = std::max(maxFraction, fraction);
            maxSigma = std::max(maxSigma, sigma);
            if (local != refLocal && isFixable(fraction, sigma, options)) ++fixedCount;
        }
        const int refOriginal = activeIndices[refLocal];
        result.logLines << QString("BIE group=%1 ref=%2 amb=%3 fixed=%4 maxFrac=%5 maxSigma=%6")
                               .arg(groupIt.key())
                               .arg(ambiguities[refOriginal].prn)
                               .arg(localIndices.size())
                               .arg(fixedCount)
                               .arg(maxFraction, 0, 'g', 12)
                               .arg(maxSigma, 0, 'g', 12);
        if (fixedCount < options.minNumSat - 1) {
            result.logLines << QString("BIE group=%1 status=reject reason=not_enough_fixable fixed=%2 min=%3")
                                   .arg(groupIt.key()).arg(fixedCount).arg(options.minNumSat - 1);
            continue;
        }
        anyEnoughGroup = true;
        if (!result.summary.isEmpty()) result.summary += QLatin1Char(' ');
        result.summary += QString("BIE%1%2=%3/%4 ref=%5")
                              .arg(ambiguities[activeIndices[localIndices.first()]].system)
                              .arg(ambiguities[activeIndices[localIndices.first()]].group)
                              .arg(fixedCount)
                              .arg(localIndices.size() - 1)
                              .arg(ambiguities[refOriginal].prn);
    }
    if (!anyEnoughGroup) {
        return result;
    }

    // The reference algorithm applies every individually fixable ambiguity once any group reaches
    // the minimum fixed-satellite threshold.
    for (auto groupIt = activeGroups.constBegin(); groupIt != activeGroups.constEnd(); ++groupIt) {
        const QVector<int>& localIndices = groupIt.value();
        const int refLocal = referenceByGroup.value(groupIt.key(), -1);
        const int firstOriginal = activeIndices[localIndices.first()];
        FixedGroup fixedGroup;
        fixedGroup.system = ambiguities[firstOriginal].system;
        fixedGroup.group = ambiguities[firstOriginal].group;
        fixedGroup.referencePrn = ambiguities[activeIndices[refLocal]].prn;
        fixedGroup.ambiguityCount = localIndices.size();
        for (int local : localIndices) {
            const double fraction = std::fabs(bie[local] - nint(bie[local]));
            const double sigma = std::sqrt(std::max(0.0, bieCovariance[local][local]));
            if (local != refLocal && isFixable(fraction, sigma, options)) {
                ++fixedGroup.fixedCount;
            }
            fixedGroup.maxFraction = std::max(fixedGroup.maxFraction, fraction);
            fixedGroup.maxSigma = std::max(fixedGroup.maxSigma, sigma);
        }
        result.fixedGroups.push_back(fixedGroup);
        for (int local : localIndices) {
            const double fraction = std::fabs(bie[local] - nint(bie[local]));
            const double sigma = std::sqrt(std::max(0.0, bieCovariance[local][local]));
            if (!isFixable(fraction, sigma, options)) continue;
            const int original = activeIndices[local];
            Constraint constraint;
            constraint.system = fixedGroup.system;
            constraint.group = fixedGroup.group;
            constraint.prn = ambiguities[original].prn;
            constraint.referencePrn = fixedGroup.referencePrn;
            constraint.stateIndex = ambiguities[original].stateIndex;
            constraint.posteriorIndex = local;
            constraint.value = bie[local];
            constraint.sigma = options.constraintSigma;
            result.constraints.push_back(constraint);
        }
    }

    if (!result.constraints.isEmpty()) {
        const int count = result.constraints.size();
        result.posteriorStateIndices.reserve(count);
        result.posteriorMean.reserve(count);
        result.posteriorCovariance = Mat(count, Vec(count, 0.0));
        for (int row = 0; row < count; ++row) {
            const int local = result.constraints[row].posteriorIndex;
            if (local < 0 || local >= bie.size() || local >= bieCovariance.size()) {
                result.constraints.clear();
                result.posteriorStateIndices.clear();
                result.posteriorMean.clear();
                result.posteriorCovariance.clear();
                result.logLines << QStringLiteral("BIE status=skip reason=invalid_posterior_index");
                return result;
            }
            result.posteriorStateIndices.push_back(result.constraints[row].stateIndex);
            result.posteriorMean.push_back(bie[local]);
            for (int col = 0; col < count; ++col) {
                const int other = result.constraints[col].posteriorIndex;
                if (other < 0 || other >= bieCovariance[local].size()) {
                    result.constraints.clear();
                    result.posteriorStateIndices.clear();
                    result.posteriorMean.clear();
                    result.posteriorCovariance.clear();
                    result.logLines << QStringLiteral("BIE status=skip reason=invalid_posterior_covariance");
                    return result;
                }
                result.posteriorCovariance[row][col] = bieCovariance[local][other];
            }
        }
    }
    return result;
}

} // namespace BieAlgorithm
