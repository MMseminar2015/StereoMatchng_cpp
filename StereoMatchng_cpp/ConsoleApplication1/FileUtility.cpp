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

	// �������s
	if (hFind == INVALID_HANDLE_VALUE)
	{
		//throw std::exception("util::Directory::GetFiles failed");
		return fileList;
	}

	do
	{
		// �t�H���_�͏���
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			continue;
		// �B���t�@�C���͏���
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_HIDDEN)
			continue;

		fileList.push_back(dirpath + fd.cFileName);
		std::cout << dirpath + fd.cFileName << std::endl;
	} while (FindNextFileA(hFind, &fd));

	FindClose(hFind);

	return fileList;

}