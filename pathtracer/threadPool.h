//-------------------------------------------------------------------------------------------------
// Toy path tracer
//-------------------------------------------------------------------------------------------------
// Copyright 2018 Carmelo J Fdez-Aguera
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// and associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#pragma once

#include <atomic>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

class ThreadPool
{
public:
	ThreadPool(size_t nWorkers)
		: mWorkers(nWorkers)
		, mMetrics(nWorkers)
	{}

	template<class Op>
	bool dispatch(size_t numTasks, const Op& operation, std::ostream& log)
	{
		// Start global profiling
		log << "Running " << mWorkers.size() << " worker threads for " << numTasks << " tasks\n";
		auto start = std::chrono::high_resolution_clock::now();

		// Reset counter and metrics
        const auto maxExpectedTasksPerThread = 2 * numTasks / mWorkers.size();
		for(auto& metric : mMetrics)
			metric.reset(maxExpectedTasksPerThread);

		// Run jobs
		mTaskCounter = 0;
		for(int i = 0; i < mWorkers.size(); ++i)
		{
			mWorkers[i] = std::thread(
				workerRoutine<Op>,
                i,
                numTasks,
				std::ref(mMetrics[i]),
				&mTaskCounter,
				std::cref(operation));

			if(!mWorkers[i].joinable())
			{
				return false;
			}
		}

		// Finish jobs
		for(auto& worker : mWorkers)
			worker.join();

		// Close global profiling
		const auto runningTime = std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::high_resolution_clock::now() - start);
		auto seconds = runningTime.count();
		log << "Running time: " << seconds << " seconds\n";

		logMetrics(seconds);

		return true;
	}

private:
	using AtomicCounter = std::atomic<size_t>;

	struct ThreadMetrics
	{
		std::vector<double> runTimes;

		void reset(size_t expectedMaxTasksPerThread)
		{
			runTimes.clear();
			runTimes.reserve(expectedMaxTasksPerThread);
		}
	};

	template<class Op>
	static void workerRoutine(size_t workerId, int numTasks, ThreadMetrics& metrics, AtomicCounter* globalCounter, const Op& operation)
	{
		assert(globalCounter);

		size_t selfCounter = (*globalCounter)++;
		while(selfCounter < numTasks) // There's still work to do, keep runing tasks
		{
			// Run task
			auto taskStart = std::chrono::high_resolution_clock::now();
			operation(selfCounter, workerId);
			std::chrono::duration<double> taskDuration = std::chrono::high_resolution_clock::now() - taskStart;
			metrics.runTimes.push_back(taskDuration.count());

			// Update task counter
			selfCounter = (*globalCounter)++;
		}
	}

	void logMetrics(double totalRunTime)
	{
		// Log raw data
		nlohmann::json log;
		log["runtime"] = totalRunTime;
		auto& threadLog = log["threads"];
		threadLog = nlohmann::json::array();
		for(auto& t : mMetrics)
			threadLog.push_back(t.runTimes);

		std::ofstream("metrics.json") << log;
	}

	AtomicCounter	mTaskCounter;
	std::vector<std::thread> mWorkers;
	std::vector<ThreadMetrics> mMetrics;
};
