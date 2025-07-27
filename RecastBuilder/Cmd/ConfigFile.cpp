#include "ConfigFile.h"
CConfigFile::CConfigFile( void )
{

}

CConfigFile::~CConfigFile( void )
{

}

bool CConfigFile::Load( std::string strFileName )
{
	FILE* pFile = fopen(strFileName.c_str(), "r+");

	if(pFile == NULL)
	{
		return false;
	}

	char szBuff[256] = {0};

	do
	{
		if (fgets(szBuff, 256, pFile) == NULL) {
			// 处理错误或文件结束
			if (feof(pFile)) {
				// 文件正常结束
			} else {
				// 读取错误（如文件损坏）
				perror("fgets failed");
				fclose(pFile);
				return false;
			}
		}

		if(szBuff[0] == ';')
		{
			continue;
		}

		char* pChar = strchr(szBuff, '=');
		if(pChar == NULL)
		{
			continue;
		}

		std::string strName;
		strName.assign(szBuff, pChar - szBuff);
		std::string strValue = pChar + 1;

		StringTrim(strName);
		StringTrim(strValue);

		m_Values.insert(std::make_pair(strName, strValue));

	}
	while(!feof(pFile));

	fclose(pFile);


	return true;
}

void CConfigFile::StringTrim(std::string& strValue)
{
	if(!strValue.empty())
	{
		strValue.erase(0, strValue.find_first_not_of((" \n\r\t")));
		strValue.erase(strValue.find_last_not_of((" \n\r\t")) + 1);
	}
}

std::string CConfigFile::GetStringValue( std::string strName )
{
	std::map<std::string, std::string>::iterator itor = m_Values.find(strName);
	if(itor != m_Values.end())
	{
		return itor->second;
	}

	printf("无效的配制选项: [%s]", strName.c_str());

	return "";
}

int CConfigFile::GetIntValue( std::string VarName )
{
	return atoi(GetStringValue(VarName).c_str());
}

float CConfigFile::GetFloatValue( std::string VarName )
{
	return (float)atof(GetStringValue(VarName).c_str());
}

double CConfigFile::GetDoubleValue( std::string VarName )
{
	return atof(GetStringValue(VarName).c_str());
}


