#include "stdafx.h"
#include "FileUtility.h"
#include <iostream>
#include <windows.h>
#include <filesystem>


std::vector<std::string> FileUtility::GetFilesFromDirectory(std::string dirpath, const std::string& filter)
{
	std::vector<std::string> fileList;

	WIN32_FIND_DATAA fd;

	std::string ss = dirpath + filter;
	HANDLE hFind = FindFirstFileA(ss.c_str(), &fd);

	// 検索失敗
	if (hFind == INVALID_HANDLE_VALUE)
	{
		//throw std::exception("util::Directory::GetFiles failed");
		return fileList;
	}

	do
	{
		// フォルダは除く
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			continue;
		// 隠しファイルは除く
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_HIDDEN)
			continue;

		fileList.push_back(dirpath + fd.cFileName);
		std::cout << dirpath + fd.cFileName << std::endl;
	} while (FindNextFileA(hFind, &fd));

	FindClose(hFind);

	return fileList;

}