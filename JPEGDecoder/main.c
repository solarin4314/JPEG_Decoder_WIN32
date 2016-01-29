#include <stdio.h>

#define JPG_FILE_PATH "c:\\jpeg\\0rgb_8x8.jpg"
#define BMP_FILE_PATH "c:\\toBmp\\0rgb_8x8.bmp"

int main(int argc, char* argv[])
{
	if(argv[1] == NULL || argv[2] == NULL)
	{
		ConvertJpgFile(JPG_FILE_PATH, BMP_FILE_PATH);
	}
	else
	{
		ConvertJpgFile(argv[1], argv[2]);
	}

	return 0;
}