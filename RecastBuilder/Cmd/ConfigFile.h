#ifndef _CONFIG_FILE_H_
#define _CONFIG_FILE_H_
#include "ShareInlude.h"
class CConfigFile
{
public:
	CConfigFile(void);
	~CConfigFile(void);

	bool Load(std::string strFileName);

	std::string GetStringValue(std::string strName);

	int GetIntValue( std::string VarName);

	float GetFloatValue( std::string VarName);

	double GetDoubleValue( std::string VarName);

	void StringTrim(std::string& strValue);

private:
	std::map<std::string, std::string> m_Values;
};

#endif