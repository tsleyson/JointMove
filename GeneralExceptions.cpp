#include "GeneralExceptions.h"

namespace TLeyson_Robot
{
	FileNotFoundException::FileNotFoundException(char* fname)
	{
		this->fname = fname;
	}

	ValueNotFoundException::ValueNotFoundException(char* fname)
	{
		this->fname = fname;
	}
}