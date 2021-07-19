#ifndef _ROCKSAMPLE_REWARD_OPTIONS_HPP_
#define _ROCKSAMPLE_REWARD_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class RocksampleRewardOptions: public PluginOptions {
public:
	using PluginOptions::PluginOptions;

	FloatType exitReward = 0.0;

	FloatType goodRockSamplingReward = 0.0;

	FloatType badRockSamplingPenalty = 0.0;

	static std::unique_ptr<options::OptionParser> makeParser() {
		std::unique_ptr<options::OptionParser> parser =
		    PluginOptions::makeParser();
		addRocksampleRewardOptions(parser.get());
		return std::move(parser);
	}

	static void addRocksampleRewardOptions(options::OptionParser* parser) {
		parser->addOption<FloatType>("rewardPluginOptions", "exitReward", &RocksampleRewardOptions::exitReward);
		parser->addOption<FloatType>("rewardPluginOptions", "goodRockSamplingReward", &RocksampleRewardOptions::goodRockSamplingReward);
		parser->addOption<FloatType>("rewardPluginOptions", "badRockSamplingPenalty", &RocksampleRewardOptions::badRockSamplingPenalty);
	}

};
}
#endif