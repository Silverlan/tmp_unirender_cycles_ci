/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#define ICYCLES_DEFINE_LOOKUP_FUNCTIONS
#include "unirender/cycles/cycles_interface.hpp"
#include <sharedutils/util_library.hpp>

#pragma optimize("",off)
icycles::CString::~CString()
{
	if(!cstr)
		return;
	delete[] cstr;
}
icycles::CString::operator std::string() const
{
	if(!cstr)
		return {};
	return std::string{cstr,len};
}
#include <sharedutils/util_path.hpp>
#include <sharedutils/util.h>
static std::shared_ptr<::util::Library> g_libInterface;
void *icycles::find_symbol(const char *name)
{
	if(!g_libInterface)
	{
		auto moduleLocation = ::util::Path::CreatePath(::util::get_program_path());
		moduleLocation += "modules/unirender/" +std::string{"cycles"} +"/";

		std::vector<std::string> additionalSearchDirectories;
		additionalSearchDirectories.push_back(moduleLocation.GetString());

		std::string libName = "icycles";
#ifdef __linux__
		libName = "lib" +libName;
#endif

		std::string err;
		g_libInterface = ::util::Library::Load(moduleLocation.GetString() +libName,additionalSearchDirectories,&err);
		g_libInterface->SetDontFreeLibraryOnDestruct();
		if(!g_libInterface)
			throw std::runtime_error{"Failed to load cycles interface library: " +err};
	}
	return g_libInterface->FindSymbolAddress(name);
}
#pragma optimize("",on)
