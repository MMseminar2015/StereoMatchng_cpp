#include "stdafx.h"
#include "FileUtility.h"
#include <iostream>
#include <windows.h>
#include <filesystem>


std::vector<std::string> FileUtility::GetFilesFromDirectory(std::string dirpath, const std::string& filter)
{
	std::vector<std::string> fileList;

	WIN32_FIND_DATAA fd;

	if(dirpath.find_last_of( "\\")!= dirpath.size()-1)
		dirpath.append("\\");

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

std::string FileUtility::AddSuffix(std::string file, std::string suffix)
{
	int index = file.find_last_of(".");
	std::string result = file.substr(0, index)+suffix+file.substr(index,file.size()-1);
	return result;
}

std::string FileUtility::GetFileName(std::string filepath)
{
	int index = filepath.find_last_of("\\");
	std::string result = filepath.substr(index + 1, filepath.size() - 1);
	return result;
}

std::string FileUtility::GetDirPath(std::string filepath)
{
	int index = filepath.find_last_of("\\");
	std::string result = filepath.substr(0, index);
	return result;
}

std::string  FileUtility::Replace(std::string String1, std::string String2, std::string String3)
{
	std::string::size_type  Pos(String1.find(String2));

	while (Pos != std::string::npos)
	{
		String1.replace(Pos, String2.length(), String3);
		Pos = String1.find(String2, Pos + String3.length());
	}

	return String1;
}