int ConvertJpgFile(char* szJpgFileInName, char * szBmpFileOutName);

const char *byte_to_binary(int x)
{
    static char b[9];
	int z;
    b[0] = '\0';

    
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
};
double IdctSum(int u)
{
	//double uu = 1.0f;
	//double vv = 1.0f;
	//if(u == 0)
	//	uu = 1.0f/sqrtf(2);
	//if(vv == 0)
	//	vv = 1.0f/sqrtf(2);

	//return (double)uu * vv;
    if (u == 0)
         return (1.0f/sqrtf(2));
    else
         return 1.0f;
};