/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2021 Silverlan */

#ifndef __CYCLES_INTERFACE_HPP__
#define __CYCLES_INTERFACE_HPP__

#include <string>

namespace icycles
{
	struct CString
	{
		char *cstr = nullptr;
		size_t len = 0;
		CString()=default;
		~CString();
		operator std::string() const;
	};

	void *find_symbol(const char *name);
	template<typename T>
		T find_symbol(const char *name) {return reinterpret_cast<T>(find_symbol(name));}
};

#include "icycles.hpp"

#endif
