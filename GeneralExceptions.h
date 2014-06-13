#ifndef GENEXCEPTIONS_H
#define GENEXCEPTIONS_H
namespace TLeyson_Robot
{
	class FileNotFoundException  { public: char* fname; FileNotFoundException(char* fname); };
	class ValueNotFoundException { public: char* fname; ValueNotFoundException(char* fname); };
}
#endif