// Checks BIE search, ambiguity screening, and posterior constraint alignment.
#include "../ambbie.h"

#include <cstdlib>
#include <cmath>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "ambbie smoke test failed: " << message << "\n";
        std::exit(1);
    }
}

BieAlgorithm::Ambiguity ambiguity(const char* prn, char system, const char* group, double value) {
    BieAlgorithm::Ambiguity out;
    out.prn = QString::fromLatin1(prn);
    out.system = QChar(system);
    out.group = QString::fromLatin1(group);
    out.stateIndex = 0;
    out.value = value;
    out.elevationDeg = 35.0;
    out.numEpochs = 20;
    return out;
}

BieAlgorithm::Mat diagonal(int size, double variance) {
    BieAlgorithm::Mat out(size, BieAlgorithm::Vec(size, 0.0));
    for (int i = 0; i < size; ++i) out[i][i] = variance;
    return out;
}

void verifyLegacyLambdaVector() {
    const BieAlgorithm::Vec floatAmbiguities = {
        -2.84908567e+04,  6.57526299e+04,  3.88303667e+04,  5.00370834e+03,
        -2.91960699e+04, -2.97658932e+02, -2.22010284e+04,  5.12358375e+04,
         3.02577810e+04,  3.89940332e+03, -2.27491854e+04, -1.59278780e+02
    };
    const double covarianceLower[] = {
         1.90688560e+04,
        -1.57839723e+04,  5.90277038e+04,
        -1.73342006e+04,  3.81426928e+04,  2.81775654e+04,
         1.44119240e+04,  5.62717388e+02, -7.00050220e+03,  1.56055082e+04,
         1.00557170e+04, -1.38300856e+04, -1.16958674e+04,  5.03970282e+03,  6.82077251e+03,
        -1.42592953e+04,  2.73734263e+04,  2.18861681e+04, -9.64896531e+03, -6.88024051e+03,  2.32465490e+04,
         1.48588484e+04, -1.22991994e+04, -1.35071695e+04,  1.12300704e+04,  7.83562345e+03, -1.11111394e+04,  1.15783237e+04,
        -1.22991994e+04,  4.59956130e+04,  2.97215786e+04,  4.38480888e+02, -1.07766903e+04,  2.13299424e+04, -9.58379157e+03,  3.58407377e+04,
        -1.35071695e+04,  2.97215786e+04,  2.19565441e+04, -5.45493698e+03, -9.11366311e+03,  1.70541567e+04, -1.05250670e+04,  2.31596718e+04,  1.71089957e+04,
         1.12300704e+04,  4.38480887e+02, -5.45493698e+03,  1.21601359e+04,  3.92704096e+03, -7.51867446e+03,  8.75070439e+03,  3.41673570e+02, -4.25060009e+03,  9.47543087e+03,
         7.83562345e+03, -1.07766903e+04, -9.11366311e+03,  3.92704096e+03,  5.31488728e+03, -5.36122657e+03,  6.10568076e+03, -8.39742084e+03, -7.10155552e+03,  3.06003207e+03,  4.14147091e+03,
        -1.11111394e+04,  2.13299424e+04,  1.70541567e+04, -7.51867446e+03, -5.36122657e+03,  1.81141936e+04, -8.65803054e+03,  1.66207345e+04,  1.32889535e+04, -5.85870722e+03, -4.17757899e+03,  1.41149564e+04
    };
    BieAlgorithm::Mat covariance(12, BieAlgorithm::Vec(12, 0.0));
    int packedIndex = 0;
    for (int row = 0; row < 12; ++row) {
        for (int col = 0; col <= row; ++col) {
            covariance[row][col] = covarianceLower[packedIndex];
            covariance[col][row] = covarianceLower[packedIndex];
            ++packedIndex;
        }
    }

    BieAlgorithm::Vec bie;
    BieAlgorithm::Mat bieCovariance;
    require(BieAlgorithm::search(floatAmbiguities, covariance, bie, bieCovariance),
            "reference implementation Lambda vector search should succeed");
    const double expected[] = {
        -28450.88, 65748.97, 38813.93, 5025.10, -29164.94, -278.05,
        -22169.91, 51232.98, 30244.95, 3916.08, -22724.96, -144.04
    };
    require(bie.size() == 12 && bieCovariance.size() == 12,
            "reference implementation Lambda vector result dimensions should match");
    for (int i = 0; i < 12; ++i) {
        require(std::fabs(bie[i] - expected[i]) < 0.02,
                "reference implementation Lambda vector estimate should match");
    }

    BieAlgorithm::Mat malformed(2);
    malformed[0] = BieAlgorithm::Vec(2, 0.0);
    malformed[1] = BieAlgorithm::Vec(1, 0.0);
    require(!BieAlgorithm::search(BieAlgorithm::Vec{1.0, 2.0}, malformed, bie, bieCovariance),
            "malformed covariance should fail without indexing past a row");
}

} // namespace

int main() {
    verifyLegacyLambdaVector();

    QVector<BieAlgorithm::Ambiguity> ambiguities;
    ambiguities << ambiguity("G01", 'G', "L1", 10.002)
                << ambiguity("G02", 'G', "L1", 11.001)
                << ambiguity("G03", 'G', "L1", 12.004)
                << ambiguity("G04", 'G', "L1", 13.000)
                << ambiguity("E01", 'E', "L1", 20.002)
                << ambiguity("E02", 'E', "L1", 21.001);

    BieAlgorithm::Options options;
    options.minNumEpochs = 10;
    options.minNumSat = 4;
    options.maxFrac = 0.01;
    options.maxSigma = 0.001;
    options.constraintSigma = 1.0e-4;
    options.candidateCount = 100;

    const BieAlgorithm::Result fixed = BieAlgorithm::run(ambiguities, diagonal(ambiguities.size(), 1.0e-6), options);
    require(fixed.fixedGroups.size() == 2, "both groups should be reported");
    require(fixed.constraints.size() == 6, "all fixable ambiguities, including references, should be constrained");
    require(fixed.posteriorStateIndices.size() == fixed.constraints.size() &&
            fixed.posteriorMean.size() == fixed.constraints.size() &&
            fixed.posteriorCovariance.size() == fixed.constraints.size(),
            "accepted constraints must retain their BIE posterior contract");
    for (int i = 0; i < fixed.constraints.size(); ++i) {
        require(fixed.posteriorStateIndices[i] == fixed.constraints[i].stateIndex,
                "BIE posterior states must remain aligned with constraints");
        require(fixed.posteriorCovariance[i].size() == fixed.constraints.size() &&
                std::isfinite(fixed.posteriorMean[i]) &&
                std::isfinite(fixed.posteriorCovariance[i][i]) &&
                fixed.posteriorCovariance[i][i] > 0.0,
                "BIE posterior covariance must be finite and positive");
    }
    require(!fixed.summary.isEmpty(), "fixed result should have a summary");

    BieAlgorithm::Options preferred = options;
    preferred.preferredReferenceByGroup.insert(QStringLiteral("G|L1"), QStringLiteral("G03"));
    const BieAlgorithm::Result preferredFixed = BieAlgorithm::run(
        ambiguities, diagonal(ambiguities.size(), 1.0e-6), preferred);
    bool gpsReferenceKept = false;
    for (const BieAlgorithm::FixedGroup& group : preferredFixed.fixedGroups) {
        if (group.system == QChar('G')) {
            gpsReferenceKept = group.referencePrn == QStringLiteral("G03");
            break;
        }
    }
    require(gpsReferenceKept, "BIE should retain a valid previous reference satellite");

    BieAlgorithm::Mat uncertain = diagonal(ambiguities.size(), 0.25);
    const BieAlgorithm::Result rejected = BieAlgorithm::run(ambiguities, uncertain, options);
    require(rejected.constraints.isEmpty(), "uncertain BIE result should fail sigma screening");
    require(rejected.posteriorStateIndices.isEmpty() && rejected.posteriorMean.isEmpty() &&
            rejected.posteriorCovariance.isEmpty(),
            "rejected BIE output must not expose a partial posterior contract");

    BieAlgorithm::Options unscreened = options;
    unscreened.maxFrac = 0.0;
    const BieAlgorithm::Result acceptedWithoutScreening = BieAlgorithm::run(ambiguities, uncertain, unscreened);
    require(acceptedWithoutScreening.constraints.size() == 6,
            "a non-positive threshold should disable both fixability checks");

    BieAlgorithm::Mat singular = diagonal(ambiguities.size(), 1.0e-6);
    singular[0][0] = 0.0;
    const BieAlgorithm::Result failed = BieAlgorithm::run(ambiguities, singular, options);
    require(failed.constraints.isEmpty(), "singular covariance should not produce constraints");

    std::cout << "ambbie smoke test passed\n";
    return 0;
}
