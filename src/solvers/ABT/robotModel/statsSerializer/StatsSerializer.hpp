#ifndef __STATS_SERIALIZER_HPP__
#define __STATS_SERIALIZER_HPP__
#include <boost/filesystem/path.hpp>
#include <math.h>
#include <map>
#include "oppt/utils/include/filesystemUtils.hpp"
#include "oppt/opptCore/core.hpp"
#include "oppt/opptCore/logging.hpp"
#include "oppt/opptCore/utils.hpp"

using std::cout;
using std::cerr;
using std::endl;


namespace oppt
{

typedef std::map<FloatType, VectorFloat>::iterator mapVecIterator;
typedef std::map<FloatType, FloatType>::iterator mapDoubleIterator;

struct SNMStats {
    SNMStats():
        robot(""),
        algorithm(""),
        snmVector(),
        processCovariance(0.0),
        observationCovariance(0.0) {

    }

    std::string robot;

    std::string algorithm;

    FloatType processCovariance;

    FloatType observationCovariance = 0.0;

    std::vector<std::map<FloatType, FloatType>> snmVector;

    void serialize(std::string& path) {
        cout << "process covariance " << processCovariance << endl;
        std::string processCovarianceRounded =
            robot::to_string_with_precision(processCovariance, 2);
        std::string observationCovarianceRounded =
            robot::to_string_with_precision(observationCovariance, 2);
        std::string file = "out_snm_" +
                           robot +
                           "_" +
                           algorithm +
                           "_" +
                           processCovarianceRounded +
                           "_" +
                           observationCovarianceRounded +
                           ".txt";

        std::string filePath = path + "/" + file;
        robot::removeFile(filePath);

        std::fstream outFile;
        outFile.open(filePath, std::fstream::out);
        for (size_t i = 0; i < snmVector.size(); i++) {
            outFile << "==========================" << endl;
            for (mapDoubleIterator iterator = snmVector[i].begin(); iterator != snmVector[i].end(); iterator++) {
                outFile << iterator->first << ": " << iterator->second << endl;
            }

        }

        outFile.close();
    }

};

struct Stats {
public:
    Stats() {

    }

    std::string robot;

    std::string algorithm;

    FloatType processCovariance = 0.0;

    FloatType observationCovariance = 0.0;

    std::shared_ptr<std::fstream> serializeInternal_(std::string& path) {
        std::string processCovarianceRounded =
            robot::to_string_with_precision(processCovariance, 2);
        std::string observationCovarianceRounded =
            robot::to_string_with_precision(observationCovariance, 2);
        std::string file = "out_" +
                           robot +
                           "_" +
                           algorithm +
                           "_" +
                           processCovarianceRounded +
                           "_" +
                           observationCovarianceRounded +
                           ".txt";
        std::string filePath = path + "/" + file;
        robot::removeFile(filePath);

        std::shared_ptr<std::fstream> outFile(new std::fstream());
        //std::fstream outFile;
        outFile->open(filePath, std::fstream::out);
        return outFile;
    }

    virtual void serialize(std::string& path) = 0;

};

struct MeasureCompareStats: public Stats {
public:
    MeasureCompareStats():
        meanSNM(0.0),
        meanNonGaussian(0.0),
        meanCurvature(0.0),
        numBodies(0) {

    }

    FloatType meanSNM;

    FloatType meanNonGaussian;

    FloatType meanCurvature;

    unsigned int numBodies;

    void serialize(std::string& path) override {
        std::shared_ptr<std::fstream> outFile = serializeInternal_(path);
        std::fstream* outFilePtr = outFile.get();
        *outFilePtr << "numBodies: " << numBodies << endl;
        *outFilePtr << "meanSNM: " << meanSNM << endl;
        *outFilePtr << "meanNonGaussian: " << meanNonGaussian << endl;
        *outFilePtr << "meanCurvature: " << meanCurvature << endl;
        outFile->close();
    }

};

struct AlgStats: public Stats {
public:
    AlgStats():
        snmMap(),
        snmStats(nullptr) {

    }

    std::shared_ptr<SNMStats> snmStats;

    VectorFloat rewardsPerRun;

    unsigned int numberOfRuns = 0;

    unsigned int numberOfSuccessfulRuns = 0;

    FloatType meanReward = 0.0;

    FloatType variance = 0.0;

    FloatType confidenceInterval;

    FloatType meanSnm = 0;

    std::map<FloatType, FloatType> snmMap;

    void serialize(std::string& path) override {
        std::shared_ptr<std::fstream> outFile = serializeInternal_(path);
        std::fstream* outFilePtr = outFile.get();
        *outFilePtr << "Robot: " << robot << endl;
        *outFilePtr << "Algorithm: " << algorithm << endl;
        *outFilePtr << "Process covariance: " << processCovariance << endl;
        *outFilePtr << "Observation covariance: " << observationCovariance << endl;
        *outFilePtr << "Number of runs: " << numberOfRuns << endl;
        *outFilePtr << "Number of successful runs: " << numberOfSuccessfulRuns << endl;
        *outFilePtr << "Mean reward: " << meanReward << endl;
        *outFilePtr << "Variance rewards: " << variance << endl;
        *outFilePtr << "Mean SNM: " << meanSnm << endl << endl;
        *outFilePtr << "MEAN_SNM_BEGIN" << endl;
        for (mapDoubleIterator iterator = snmMap.begin(); iterator != snmMap.end(); iterator++) {
            *outFilePtr << iterator->first << ": " << iterator->second << endl;
        }
        *outFilePtr << "MEAN_SNM_END" << endl;
        *outFilePtr << "REWARDS_START" << endl;
        for (auto & k : rewardsPerRun) {
            *outFilePtr << k << endl;
        }
        *outFilePtr << "REWARDS_END" << endl;
        outFile->close();
        if (snmStats) {
            snmStats->serialize(path);
        }

    }

};

struct RandomBodiesStats: public Stats {
public:
    RandomBodiesStats() {

    }

    FloatType discountedReward = 0.0;

    bool successfulRun = false;

    void serialize(std::string& path) override {

    }

};

typedef std::shared_ptr<Stats> StatsPtr;
typedef std::vector<StatsPtr> VectorStats;
typedef std::map<std::string, VectorStats> AlgorithmMap;

class MultipleStats
{
public:
    MultipleStats():
        stats_() {

    }

    void addStats(const StatsPtr& stats) {
        stats_.push_back(stats);
    }

    void serialize(std::string& path) {
        std::string file("multipleStats.txt");
        std::string filePath = path + "/" + file;
        robot::removeFile(filePath);
        AlgorithmMap algMap = makeAlgorithmMap();
        std::fstream outFile;
        cout << "Write stats to file '" << filePath << "'" << endl;
        outFile.open(filePath, std::fstream::out);
        for (auto & entry : algMap) {
            cout << "key: " << entry.first << endl;
            if (entry.second.size() > 0) {
                outFile << "Algorithm: " << entry.first << "\n";
                outFile << "Robot: " << entry.second[0]->robot << "\n";
                outFile << "Process covariance: " << entry.second[0]->processCovariance << "\n";
                outFile << "Observation covariance: " << entry.second[0]->observationCovariance << "\n";
                outFile << "Number of runs: " << entry.second.size() << "\n";
                unsigned int numSuccessfulRuns = getNumberOfSuccessfulRuns_(entry.second);
                outFile << "Number of successful runs: " << numSuccessfulRuns << " \n";
                outFile << "Percentage of successful runs: " << (100.0 / entry.second.size()) * numSuccessfulRuns << "\n";
                outFile << "Mean discounted reward: " << getMeanDiscountedReward_(entry.second) << "\n";
                outFile << "ALL REWARDS \n";
                for (auto & stat : entry.second) {
                    outFile << static_cast<RandomBodiesStats*>(stat.get())->discountedReward << " \n";
                }
                outFile << "END ALL REWARDS \n";
            }
        }

        /**if (stats_.size() == 0) {
            ERROR("No stats provided");
        }

        outFile << "Process covariance: " << stats_[0]->processCovariance << "\n";
        outFile << "Observation covariance: " << stats_[0]->observationCovariance << "\n";
        outFile << "Number of runs: " << stats_.size() << "\n";
        outFile << "Mean discounted reward: " << getMeanDiscountedReward_() << "\n";*/
        outFile.close();
    }

private:
    VectorStats stats_;

    AlgorithmMap makeAlgorithmMap() {
        AlgorithmMap map;
        for (auto & stat : stats_) {
            auto it = map.find(stat->algorithm);
            if (it != map.end()) {
                map.at(stat->algorithm).push_back(stat);
            } else {
                map[stat->algorithm] = VectorStats( {stat});
            }
        }

        return map;
    }

    FloatType getMeanDiscountedReward_(const VectorStats& stats) {
        FloatType meanDiscountedReward = 0;
        for (auto & stat : stats) {
            meanDiscountedReward += static_cast<RandomBodiesStats*>(stat.get())->discountedReward;
        }

        meanDiscountedReward /= stats.size();
        return meanDiscountedReward;
    }

    unsigned int getNumberOfSuccessfulRuns_(const VectorStats& stats) {
        unsigned int numSuccessfulRuns = 0;
        for (auto & stat : stats) {
            if (static_cast<RandomBodiesStats*>(stat.get())->successfulRun) {
                numSuccessfulRuns++;
            }
        }

        return numSuccessfulRuns;
    }

};

class StatsSerializer
{
public:
    StatsSerializer():
        path_(),
        logFiles_() {

    }

    void setPath(std::string& path) {
        path_ = path;
    }

    void readLogfiles() {
        cout << "Generate statistics from log files..." << endl;

        parseRandomBodyLogFiles(path_);
        //stats->serialize(path_);
        return;


        /**for (auto i = boost::filesystem::directory_iterator(p);
                i != boost::filesystem::directory_iterator(); i++) {
            std::string logFile = i->path().filename().string();
            cout << "logFile: " << logFile << endl;

            if (logFile.find(".log") != std::string::npos) {
                std::string pathToLogFile = path_ + "/" + logFile;
                std::shared_ptr<Stats> stats = parseLogFile(pathToLogFile);
                stats->serialize(path_);
            }
        }
        cout << "Done." << endl;*/
    }

    void parseRandomBodyLogFiles(std::string& path) {
        boost::filesystem::path p(path);
        MultipleStats multipleStats;
        int n = 0;
        for (auto i = boost::filesystem::directory_iterator(p);
                i != boost::filesystem::directory_iterator(); i++) {
            std::string logFile = i->path().filename().string();
            if (logFile.find(".log") != std::string::npos) {
		cout << "process: " << logFile << endl;
                std::shared_ptr<Stats> stats = std::make_shared<RandomBodiesStats>();
                std::string pathToLogFile = path + "/" + logFile;
                std::string robot = findRobot(pathToLogFile);
                if (robot == "") {
                    WARNING("No robot found in log file '" + pathToLogFile + "'. Cannot generate stats");
                    continue;
                }

                stats->robot = robot;
                std::string algorithm = findAlgorithm(pathToLogFile);
                if (algorithm == "") {
                    ERROR("No algorithm found in log file '" + pathToLogFile + "'. Cannot generate stats");
                    continue;
                }

                stats->algorithm = algorithm;
                if (logFileValid(pathToLogFile)) {
                    stats->processCovariance = findProcessCovariance(pathToLogFile);
                    stats->observationCovariance = findObservationCovariance(pathToLogFile);
                    static_cast<RandomBodiesStats*>(stats.get())->discountedReward = findDiscountedReward(pathToLogFile);
                    static_cast<RandomBodiesStats*>(stats.get())->successfulRun = findSuccessfulRun(pathToLogFile);
                    multipleStats.addStats(stats);
                } else {
		    WARNING("Log file '" + pathToLogFile + "' not valid. Skipping");
		}
            }
        }

        multipleStats.serialize(path);
    }

    bool findSuccessfulRun(std::string& logFile) {
        bool success = false;
        std::ifstream input(logFile);
        for (std::string line; getline(input, line);) {
            if (line.find("Num successes:") != std::string::npos) {
                if (line.find("1") != std::string::npos) {
                    success = true;
                }
            }
        }
        return success;
    }

    FloatType findDiscountedReward(std::string& logFile) {
        FloatType discountedReward = 0.0;
        std::ifstream input(logFile);
        for (std::string line; getline(input, line);) {
            if (line.find("Total discounted reward:") != std::string::npos) {
                VectorString procElems;
                split(line, ': ', procElems);
                discountedReward = atof(procElems[3].c_str());
            }

        }
        return discountedReward;
    }

    bool logFileValid(std::string& logFile) {
        std::ifstream input(logFile);
        for (std::string line; getline(input, line);) {
            if (line.find("Percentage of successful runs:") != std::string::npos) {
                return true;
            }
        }

        return false;
    }

    FloatType findProcessCovariance(std::string& logFile) {
        FloatType processCovariance = 0.0;
        std::ifstream input(logFile);
        for (std::string line; getline(input, line);) {
            if (line.find("Process covariance:") != std::string::npos) {
                VectorString procElems;
                split(line, ': ', procElems);
                processCovariance = atof(procElems[2].c_str());
            }

        }
        return processCovariance;
    }

    FloatType findObservationCovariance(std::string& logFile) {
        FloatType observationCovariance = 0.0;
        std::ifstream input(logFile);
        for (std::string line; getline(input, line);) {
            if (line.find("Observation covariance:") != std::string::npos) {
                VectorString procElems;
                split(line, ': ', procElems);
                observationCovariance = atof(procElems[2].c_str());
            }

        }
        return observationCovariance;
    }

    std::string findRobot(std::string& logFile) {
        std::string robot = "";
        std::ifstream input(logFile);
        for (std::string line; getline(input, line);) {
            if (line.find("Robot: ") != std::string::npos) {
                VectorString robotElems;
                split(line, ':', robotElems);
                robot = robotElems[1];
            }
        }

        input.close();
        return robot;
    }

    std::string findAlgorithm(std::string& logFile) {
        std::string algorithm = "";
        std::ifstream input(logFile);
        for (std::string line; getline(input, line);) {
            if (line.find("algorithm: ") != std::string::npos) {
                VectorString algorithmElems;
                split(line, ':', algorithmElems);
                algorithm = algorithmElems[1];
            }
        }

        input.close();
        return algorithm;
    }

    std::shared_ptr<Stats> parseLogFile(std::string& path) {
        std::shared_ptr<Stats> stats;
        std::vector<std::string> elems;
        std::vector<std::string> fileElems;
        split(path, '/', elems);
        split(elems[elems.size() - 1], '_', fileElems);

        std::string robotString = fileElems[1];
        std::string algorithmString = fileElems[2];
        cout << "alg string: " << algorithmString << endl;
        FloatType processCovariance = atof(fileElems[3].c_str());
        std::size_t found = fileElems[4].find(".log");
        FloatType observationCovariance = atof(fileElems[4].erase(found).c_str());

        if (algorithmString.find("measureCompareObst") != std::string::npos) {
            stats = std::make_shared<MeasureCompareStats>();
            stats->algorithm = algorithmString;
            MeasureCompareStats* measureCompareStats = static_cast<MeasureCompareStats*>(stats.get());
            unsigned int numBodies = readNumBodies(path);
            measureCompareStats->numBodies = numBodies;
            cout << "numBodies: " << numBodies << endl;
            VectorFloat meanMeasures = readMeanMeasures(path);
            if (meanMeasures.size() > 0) {
                measureCompareStats->meanSNM = meanMeasures[0];
            }

            if (meanMeasures.size() > 1) {
                measureCompareStats->meanNonGaussian = meanMeasures[1];
            }

            if (meanMeasures.size() > 2) {
                measureCompareStats->meanCurvature = meanMeasures[2];
            }

        } else {
            stats = std::make_shared<AlgStats>();
            stats->algorithm = algorithmString;
            AlgStats* algStats = static_cast<AlgStats*>(stats.get());
            algStats->numberOfRuns = readNumberOfRuns(path);
            algStats->numberOfSuccessfulRuns = readNumberOfSuccessfulRuns(path);
            algStats->rewardsPerRun = parseRewardsPerRun(path);
            FloatType sumRewards = 0;
            for (auto & k : algStats->rewardsPerRun) {
                sumRewards += k;
            }

            algStats->meanReward = sumRewards / (FloatType)algStats->numberOfRuns;
            algStats->variance = calcVariance(algStats->rewardsPerRun);
            cout << "path: " << path << endl;
            cout << "numRuns: " << algStats->numberOfRuns << endl;
            cout << "size: " << algStats->rewardsPerRun.size() << endl;

            VectorFloat snmPerRun = parseSNMPerRun(path);
            FloatType meanSNM = 0;
            for (auto & k : snmPerRun) {
                meanSNM += k;
            }

            algStats->meanSnm = meanSNM / (FloatType)algStats->numberOfRuns;
            makeSNMMap(path, algStats);
            FloatType conf = 0.95;

            cout << "alg: " << algStats->algorithm << endl;
            if (algStats->algorithm == "nnl") {
                cout << "MAKE SNM STATS" << endl;
                algStats->snmStats = std::make_shared<SNMStats>();
                algStats->snmStats->robot = algStats->robot;
                algStats->snmStats->algorithm = algStats->algorithm;
                algStats->snmStats->processCovariance = algStats->processCovariance;
                algStats->snmStats->observationCovariance = algStats->observationCovariance;
                makeSNMStats(path, algStats);
            }
        }

        stats->robot = robotString;
        stats->processCovariance = processCovariance;
        stats->observationCovariance = observationCovariance;
        return stats;
    }

    std::string getCurrentDirectory() {
        char* buffer = getcwd(nullptr, 0);
        if (buffer == nullptr) {
            cerr << "ERROR: Failed to get current path." << endl;
            std::exit(4);
        }
        std::string dir(buffer);
        free(buffer);
        return dir;
    }

private:
    std::string path_;

    std::vector<std::string> logFiles_;

    std::vector<std::string> split(const std::string& s, char delim, std::vector<std::string>& elems) {
        std::stringstream ss(s);
        std::string item;
        while (getline(ss, item, delim)) {
            elems.push_back(item);
        }
        return elems;
    }

    void makeSNMStats(std::string& file, AlgStats* stats) {
        SNMStats* snmStats = stats->snmStats.get();
        snmStats->snmVector.clear();
        std::ifstream input(file);
        std::map<FloatType, FloatType> snmMap;
        for (std::string line; getline(input, line);) {
            if (line.find("MoN") != std::string::npos) {
                if (line.find("MoN total") != std::string::npos) {
                    snmStats->snmVector.push_back(snmMap);
                    snmMap = std::map<FloatType, FloatType>();

                } else {
                    if (line.find("MoN ") == std::string::npos && line.find("MoN:") == std::string::npos) {
                        std::size_t found = line.find("MoN");
                        line.erase(0, 3);
                        std::vector<std::string> elems;
                        split(line, ':', elems);
                        FloatType key = atof(elems[0].c_str());
                        std::vector<std::string> elems2;
                        split(elems[1], ' ', elems2);
                        FloatType snm = atof(elems2[1].c_str());
                        snmMap[key] = snm;
                    }
                }

            }
        }

        input.close();
    }

    void makeSNMMap(std::string& file, AlgStats* stats) {
        std::map<FloatType, VectorFloat> snmMapAll;
        std::ifstream input(file);
        for (std::string line; getline(input, line);) {
            if (line.find("MoN") != std::string::npos &&
                    line.find("MoN:") == std::string::npos &&
                    line.find("MoN ") == std::string::npos) {
                std::size_t found = line.find("MoN");
                line.erase(0, 3);
                std::vector<std::string> elems;
                split(line, ':', elems);
                FloatType key = atof(elems[0].c_str());
                if (snmMapAll.count(key) == 0) {
                    snmMapAll[key] = VectorFloat();
                }

                std::vector<std::string> elems2;
                split(elems[1], ' ', elems2);
                FloatType snm = atof(elems2[1].c_str());
                snmMapAll[key].push_back(snm);
            }
        }
        for (mapVecIterator iterator = snmMapAll.begin(); iterator != snmMapAll.end(); iterator++) {
            auto key = iterator->first;
            VectorFloat snms(iterator->second);
            FloatType mean = 0;
            for (size_t i = 0; i < snms.size(); i++) {
                mean += snms[i];
            }

            mean /= (FloatType)snms.size();
            stats->snmMap[key] = mean;
        }

        input.close();
    }

    unsigned int readNumBodies(std::string& path) {
        std::ifstream input(path);
        unsigned int numBodies = 0;
        for (std::string line; getline(input, line);) {
            if (line.find("numBodies") != std::string::npos) {
                std::vector<std::string> elems;
                split(line, ' ', elems);
                oppt::printVector<std::string>(elems, "elems");
                numBodies = (unsigned int)atof(elems[elems.size() - 1].c_str());
                return numBodies;
            }
        }

        return numBodies;
    }

    VectorFloat readMeanMeasures(std::string& path) {
        std::ifstream input(path);
        VectorFloat meanMeasures;
        VectorFloat allSNM;
        VectorFloat allNonGaussian;
        VectorFloat allCurvature;
        for (std::string line; getline(input, line);) {
            if (line.find("monSNM") != std::string::npos) {
                std::vector<std::string> elems;
                split(line, ' ', elems);
                FloatType snm = atof(elems[elems.size() - 1].c_str());
                if (snm != 0.0) {
                    allSNM.push_back(snm);
                }

            } else if (line.find("monNonGaussian") != std::string::npos) {
                std::vector<std::string> elems;
                split(line, ' ', elems);
                FloatType snm = atof(elems[elems.size() - 1].c_str());
                if (snm != 0.0) {
                    allNonGaussian.push_back(snm);
                }

            } else if (line.find("monCurvature") != std::string::npos) {
                std::vector<std::string> elems;
                split(line, ' ', elems);
                FloatType snm = atof(elems[elems.size() - 1].c_str());
                if (snm != 0.0) {
                    allCurvature.push_back(snm);
                }
            }
        }

        FloatType sum = 0.0;
        for (size_t i = 0; i < allSNM.size(); i++) {
            sum += allSNM[i];
        }

        meanMeasures.push_back(sum / (FloatType)(allSNM.size()));
        sum = 0.0;
        for (size_t i = 0; i < allNonGaussian.size(); i++) {
            sum += allNonGaussian[i];
        }

        meanMeasures.push_back(sum / (FloatType)(allNonGaussian.size()));
        sum = 0.0;
        for (size_t i = 0; i < allCurvature.size(); i++) {
            sum += allCurvature[i];
        }

        meanMeasures.push_back(sum / (FloatType)(allCurvature.size()));
        return meanMeasures;
    }

    unsigned int readNumberOfRuns(std::string& file) const {
        std::ifstream input(file);
        unsigned int counter = 0;
        bool firstRun = true;
        for (std::string line; getline(input, line);) {
            if (line.find("Run #") != std::string::npos ||
                    line.find("#####") != std::string::npos) {
                if (!firstRun) {
                    counter++;
                }

                firstRun = false;
            }
        }

        input.close();
        return counter;
    }

    unsigned int readNumberOfSuccessfulRuns(std::string& file) {
        std::ifstream input(file);
        unsigned int numSuccRuns = 0;
        for (std::string line; getline(input, line);) {
            if (line.find("Num successes:") != std::string::npos) {
                VectorString lineElems;
                split(line, ': ', lineElems);
                numSuccRuns = atof(lineElems[lineElems.size() - 1].c_str());
            }
        }

        input.close();
        return numSuccRuns;
    }

    VectorFloat parseRewardsPerRun(std::string& file) {
        VectorFloat rewardsPerRun;
        FloatType reward = 0;
        FloatType minReward = 1000000.0;
        FloatType maxReward = -1000000.0;
        std::ifstream input(file);
        for (std::string line; getline(input, line);) {
            if (line.find("DISCOUNTED_REWARD") != std::string::npos) {
                std::vector<std::string> elems;
                split(line, ': ', elems);
                reward += atof(elems[elems.size() - 1].c_str());
            }

            if (line.find("END_RUN") != std::string::npos) {
                if (reward < minReward) {
                    minReward = reward;
                }

                if (reward > maxReward) {
                    maxReward = reward;
                }
                rewardsPerRun.push_back(reward);
                cout << line << ": reward: " << reward << endl;
                reward = 0;
            }
        }

        cout << "minReward: " << minReward << endl;
        cout << "maxReward: " << maxReward << endl;

        input.close();
        return rewardsPerRun;
    }

    VectorFloat parseSNMPerRun(std::string& file) {
        VectorFloat snmPerRun;
        FloatType s = 0;
        FloatType snm = 0;
        FloatType numSteps = 0;
        std::ifstream input(file);
        for (std::string line; getline(input, line);) {
            if (line.find("Run #") != std::string::npos ||
                    line.find("#####") != std::string::npos) {
                if (line.find("Run #1") == std::string::npos) {
                    FloatType snmMean = snm / numSteps;
                    snmPerRun.push_back(snmMean);
                    snm = 0;
                    numSteps = 0;
                }
            }
            if ((line.find("MoN total") != std::string::npos ||
                    line.find("MoN:") != std::string::npos) && line.find("time") == std::string::npos) {
                std::vector<std::string> elems;
                split(line, ' ', elems);
                s = atof(elems[elems.size() - 1].c_str());
                snm += s;
                numSteps += 1;
            }
        }
        input.close();
        return snmPerRun;
    }

    FloatType calcMean(VectorFloat& rewards) const {
        FloatType sum = 0;
        for (auto & k : rewards) {
            sum += k;
        }

        return sum / rewards.size();
    }

    FloatType calcVariance(VectorFloat& rewards) const {
        FloatType mean = calcMean(rewards);
        FloatType sum = 0.0;
        for (auto & k : rewards) {
            sum += std::pow(k - mean, 2);
        }

        return sum / rewards.size();
    }

    FloatType calcStandardDeviation(VectorFloat& rewards) const {
        FloatType variance = calcVariance(rewards);
        return sqrt(variance);
    }

    FloatType calcConfidenceInterval(VectorFloat& rewards, FloatType& confidence) const {
        unsigned int dof = 9;
        FloatType t = 0.025;
        FloatType gam = tPdf(dof, t);
        cout << "gamma: " << gam << endl;
        sleep(100);


    }

    FloatType tPdf(unsigned int& dof, FloatType& t) const {
        FloatType upper = lgamma((dof + 1.0) / 2.0);
        FloatType lower = sqrt(dof * M_PI) * lgamma(dof / 2.0);
        FloatType power = -((dof + 1) / 2.0);
        FloatType second = std::pow(1 + (t * t / dof), power);
        return (upper / lower) * second;

    }

};
}

#endif


