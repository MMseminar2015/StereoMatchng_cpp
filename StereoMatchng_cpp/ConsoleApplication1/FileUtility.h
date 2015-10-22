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
};

