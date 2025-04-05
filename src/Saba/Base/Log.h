//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

#ifndef SABA_BASE_LOG_H_
#define SABA_BASE_LOG_H_

#include "Singleton.h"

#include <memory>
#include <algorithm>
#include <assert.h>

namespace saba
{
	
	class Logger
	{
	public:
		Logger()
		{
			
		}

		template <typename... Args>
		void Info(const char* message, const Args&... args)
		{
			
		}

		template <typename... Args>
		void Warn(const char* message, const Args&... args)
		{
			
		}

		template <typename... Args>
		void Error(const char* message, const Args&... args)
		{
			
		}
	};

	template <typename... Args>
	void Info(const char* message, const Args&... args)
	{
		Singleton<Logger>::Get()->Info(message, args...);
	}

	template <typename... Args>
	void Warn(const char* message, const Args&... args)
	{
		Singleton<Logger>::Get()->Warn(message, args...);
	}

	template <typename... Args>
	void Error(const char* message, const Args&... args)
	{
		Singleton<Logger>::Get()->Error(message, args...);
	}
}

#define SABA_INFO(message, ...)\
	saba::Info(message, ##__VA_ARGS__)

#define SABA_WARN(message, ...)\
	saba::Warn(message, ##__VA_ARGS__)

#define SABA_ERROR(message, ...)\
	saba::Error(message, ##__VA_ARGS__)

#define SABA_ASSERT(expr)\
	assert(expr)

#endif // !SABA_BASE_LOG_H_

