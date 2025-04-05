//
// Copyright(c) 2016-2017 benikabocha.
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//

//
// Copyright(c) HanetakaChou(YuqiaoZhang).
// Distributed under the LGPL License (https://opensource.org/license/lgpl-2-1)
//

#ifndef SABA_MODEL_MMD_VMDFILE_H_
#define SABA_MODEL_MMD_VMDFILE_H_

#include "MMDFileString.h"

#include <string>
#include <vector>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <array>
#include <cstdint>

#ifndef MCRT
#define MCRT 1
using mcrt_string = std::string;

template <typename T>
using mcrt_vector = std::vector<T>;

template <typename Key, typename Compare = std::less<Key>>
using mcrt_set = std::set<Key, Compare>;

template <typename Key, typename T, typename Compare = std::less<Key>>
using mcrt_map = std::map<Key, T, Compare>;

template <typename Key>
using mcrt_unordered_set = std::unordered_set<Key, std::hash<Key>, std::equal_to<Key>>;

template <typename Key, typename T>
using mcrt_unordered_map = std::unordered_map<Key, T, std::hash<Key>, std::equal_to<Key>>;
#endif

namespace saba
{
	struct VMDFile
	{
	};

	bool ReadVMDFile(VMDFile *vmd, const char *filename);
}

#endif // !SABA_MODEL_MMD_VMDFILE_H_
