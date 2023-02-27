#pragma once

//#define D_YAGPSR_DISABLE_MULTITHREADING

#ifndef D_YAGPSR_DISABLE_MULTITHREADING
#ifdef _MSC_VER
#include <ppl.h>
#else
#include <thread>
#endif
#endif

#ifdef D_YAGPSR_DISABLE_MULTITHREADING
#define begin_yagpsr_parallel_for_each(prlvar, prlval_type) 					\
	for (size_t prln = 0; prln < prlvar.size(); prln++)						\
	{																			\
		prlval_type = prlvar[prln];												
#else
#ifdef D_YAGPSR_USE_OPENMP
#define begin_yagpsr_parallel_for_each(prlvar, prlval_type) 					\
	OMP_PRAGMA                                                                  \
    for (size_t prln = 0; prln < prlvar.size(); prln++)							\
	{																			\
		prlval_type = prlvar[prln];
#else
#ifdef _MSC_VER															
#define begin_yagpsr_parallel_for_each(prlvar, prlval_type) {					\
	concurrency::parallel_for_each(begin(prlvar), end(prlvar), [&](prlval_type)	\
	{																	
#else																	
#define begin_yagpsr_parallel_for_each(prlvar, prlval_type) {			\
	std::vector<std::thread> threads;									\
																		\
	auto prlpersist = prlvar;											\
	auto DSLoop = [&](prlval_type) {
#endif
#endif
#endif

#ifdef D_YAGPSR_DISABLE_MULTITHREADING
#define end_yagpsr_parallel_for_each	}
#else
#ifdef D_YAGPSR_USE_OPENMP 
#define end_yagpsr_parallel_for_each	}
#else
#ifdef _MSC_VER															
#define end_yagpsr_parallel_for_each									\
	});																	\
	}
#else																	
#define end_yagpsr_parallel_for_each									\
	};																	\
																		\
	for (size_t n = 0; n < prlpersist.size(); n++)					\
	{																	\
		threads.push_back(std::thread(DSLoop, prlpersist[n]));			\
	}																	\
																		\
	for (auto &thread : threads)										\
	{																	\
		thread.join();													\
	}																	\
}																		
#endif
#endif
#endif
