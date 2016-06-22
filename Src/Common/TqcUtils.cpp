/**
@ TqcUtils.cpp
@ utility functions.
@ v1.0 2016.2.17 by Benet Huang
*/

#ifdef WIN32
#include <Windows.h>
#endif
#include "TqcUtils.h"

using namespace std;

CLock::CLock()
{
    Init();
}
CLock::~CLock()
{
    UnInit();
}

bool CLock::Lock()
{
    TqcOsAcquireMutex(m_handle);
    return true;
}
bool CLock::UnLock()
{
    TqcOsReleaseMutex(m_handle);
    return true;
}

bool CLock::Init()
{
    m_handle = TqcOsCreateMutex();
    if (!m_handle)
    {
        return false;
    }

    return true;
}

bool CLock::UnInit()
{
    TqcOsDeleteMutex(m_handle);
    return true;
}

#ifdef WIN32
void AddFileList(const char *lpPath, const char *filePrefix, vector<char*> &fileList)
{
    char            szFind[TQC_MAX_PATH];
    WIN32_FIND_DATA FindFileData;

    strcpy(szFind, lpPath);
    strcat(szFind, "*.*");

    HANDLE hFind = ::FindFirstFile(szFind, &FindFileData);

    if (INVALID_HANDLE_VALUE == hFind)
        return;

    while (TRUE)
    {
        if (FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
        {
        }
        else
        {
            if (strstr(FindFileData.cFileName, filePrefix))
            {
                char *szFile;
                szFile = (char*)malloc(TQC_MAX_PATH);

                memset(szFile, 0, TQC_MAX_PATH);
                strcpy(szFile, lpPath);
                strcat(szFile, "");
                strcat(szFile, FindFileData.cFileName);
                fileList.push_back(szFile);
            }
        }

        if (!FindNextFile(hFind, &FindFileData))
            break;
    }

    FindClose(hFind);
}

void AddFileList(const char *filePrefix, vector<char*> &fileList)
{
    char   path[TQC_MAX_PATH];
    char   filePre[TQC_MAX_PATH];
    size_t len = strlen(filePrefix) - 1;
    size_t orgLen = len;

    memset(path, 0, TQC_MAX_PATH);
    memset(filePre, 0, TQC_MAX_PATH);

    while (filePrefix[len] != '\\')
    {
        len--;
    }

    memcpy(path, filePrefix, len + 1);
    memcpy(filePre, &filePrefix[len + 1], orgLen - len);

    AddFileList(path, filePre, fileList);
}
#endif