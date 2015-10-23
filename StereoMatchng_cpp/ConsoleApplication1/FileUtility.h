#pragma once

#include "string.h"
#include "stdafx.h"
#include "vector"

class FileUtility
{
public:
	/// <summary>
	/// ディレクトリ内のファイルを取得
	/// </summary>
	static std::vector<std::string> GetFilesFromDirectory(std::string dirpath, const std::string& filter);

	static std::string AddSuffix(std::string file, std::string suffix);
	static std::string GetFileName(std::string filepath);
	static std::string GetDirPath(std::string filepath);
	static std::string Replace(std::string String1, std::string String2, std::string String3);
};

