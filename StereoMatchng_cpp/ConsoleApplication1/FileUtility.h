#pragma once

#include "string.h"
#include "stdafx.h"
#include "vector"

class FileUtility
{
public:
	/// <summary>
	/// �f�B���N�g�����̃t�@�C�����擾
	/// </summary>
	static std::vector<std::string> GetFilesFromDirectory(std::string dirpath, const std::string& filter);
};

