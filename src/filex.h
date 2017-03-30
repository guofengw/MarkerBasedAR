#ifndef FILEX_H
#define FILEX_H

#include <windows.h>
#include <vector>
#include <string>

using std::string;
using std::vector;

int GetFileNames(const string &directoryPath, vector<string> &fileNames)
{
	WIN32_FIND_DATA data;  
	HANDLE hFind;   

	string allNameAndType = directoryPath + "/*" ;

	hFind = FindFirstFile(allNameAndType.c_str(), &data);
	while( hFind!=INVALID_HANDLE_VALUE )   
	{   
		if(data.cFileName[0] != '.' && data.dwFileAttributes != FILE_ATTRIBUTE_DIRECTORY )
		{
			fileNames.push_back( string(data.cFileName) ); 
		}      
		if(!FindNextFile(hFind,&data))   
		{     
			hFind=INVALID_HANDLE_VALUE;   
		}   
	}

	FindClose(hFind);
	return   1;   
}


int GetSubDirNames(const string &directoryPath, vector<string> &dirNames)
{
	WIN32_FIND_DATA data;  
	HANDLE hFind;   

	string allNameAndType = directoryPath + "/*" ;

	hFind = FindFirstFile(allNameAndType.c_str(), &data);
	while( hFind!=INVALID_HANDLE_VALUE )   
	{   
		if(data.cFileName[0] != '.' && data.dwFileAttributes == FILE_ATTRIBUTE_DIRECTORY )
		{
			dirNames.push_back( string(data.cFileName) ); 
		}      
		if(!FindNextFile(hFind,&data))   
		{     
			hFind=INVALID_HANDLE_VALUE;   
		}   
	}

	FindClose(hFind);
	return   1;   
}

#endif //FILEX_H