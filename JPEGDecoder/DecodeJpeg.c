#include <stdio.h>		// sprintf(..), fopen(..)
#include <stdlib.h>		// strtol...
#include <stdarg.h>     // So we can use ... (in dprintf)
#include <string.h>		// memset(..), strcat...
#include <math.h>		// sqrt(..), cos(..)
#include "DecodeJpeg.h"



// Takes the rgb pixel values and creates a bitmap file
void WriteBMP24(const char* szBmpFileName, int Width, int Height, unsigned char* RGB);

// Pass in the whole jpg file from memory, and it decodes it to RGB pixel data
void DecodeJpgFileData(const unsigned char* buf, const int sizeBuf, unsigned char** rgbpix, unsigned int* width, unsigned int* height);
// Don't forget to delete[] rgbpix if you use DecodeJpgFileData..

int JpegParseHeader(struct stJpegData *jdata, const unsigned char *buf, unsigned int size);

void DecodeMCU(struct stJpegData *jdata, int w, int h);
void YCrCB_to_RGB24_Block8x8(struct stJpegData *jdata, int w, int h, int imgx, int imgy, int imgw, int imgh);
void ConvertYCrCbtoRGB(int y, int cb, int cr, int* r, int* g, int* b);
void ProcessHuffmanDataUnit(struct stJpegData *jdata, int indx);
void DecodeSingleBlock(struct stComponent *comp, unsigned char *outputBuf, int stride);

void DumpDecodedBlock(int val[8][8]);

void PerformIDCT(int outBlock[8][8], const int inBlock[8][8]);
void DequantizeBlock( int block[64], const float quantBlock[64]);
void DeZigZag(int outBlock[64], const int inBlock[64]);
void TransformArray(int outArray[8][8], const int inArray[64]);

void DumpDCTValues(short dct[64]);
int IDCTCalculate(int x, int y, const int block[8][8]);

void FillNBits(const unsigned char** stream, int nbits_wanted);
short GetNBits(const unsigned char** stream, int nbits_wanted);
int LookNBits(const unsigned char** stream, int nbits_wanted);
void SkipNBits(const unsigned char** stream, int nbits_wanted);

int DetermineSign(int val, int nBits);

void BuildHuffmanTable(const unsigned char *bits, const unsigned char *stream, struct stHuffmanTable *HT);

void JpegGetImageSize(struct stJpegData *jdata, unsigned int *width, unsigned int *height);
void BuildQuantizationTable(float *qtable, const unsigned char *ref_table);
void GenHuffCodes( int num_codes, struct stBlock* arr, unsigned char* huffVal );

void DrawBlockData(unsigned char *buffer, int mcu_x, int mcu_y, int x, int y);

#define DQT 	 0xDB	// Define Quantization Table
#define SOF 	 0xC0	// Start of Frame (size information)
#define DHT 	 0xC4	// Huffman Table
#define SOI 	 0xD8	// Start of Image
#define SOS 	 0xDA	// Start of Scan
#define EOI 	 0xD9	// End of Image, or End of File
#define APP0	 0xE0

#define BYTE_TO_WORD(x) (((x)[0]<<8)|(x)[1])


#define HUFFMAN_TABLES		2
#define COMPONENTS			4

#define cY	1
#define cCb	2
#define cCr	3
// ������� ��ĵ
static int ZigZagArray[64] = 
{
	0,   1,   5,  6,   14,  15,  27,  28,
	2,   4,   7,  13,  16,  26,  29,  42,
	3,   8,  12,  17,  25,  30,  41,  43,
	9,   11, 18,  24,  31,  40,  44,  53,
	10,  19, 23,  32,  39,  45,  52,  54,
	20,  22, 33,  38,  46,  51,  55,  60,
	21,  34, 37,  47,  50,  56,  59,  61,
	35,  36, 48,  49,  57,  58,  62,  63,
};

struct stBlock
{
	int value;					// Decodes to.
	int length;				// Length in bits.
	unsigned short int code;	// 2 byte code (variable length)
};
// AC HuffmanTable
struct stHuffmanTable
{
	unsigned char	m_length[17];		// 17 values from jpg file, 
	// k =1-16 ; L[k] indicates the number of Huffman codes of length k
	unsigned char	m_hufVal[257];		// 256 codes read in from the jpeg file

	int				m_numBlocks;
	struct stBlock			m_blocks[256];
};
// DC HuffmanTable
struct stHuffmanTable2
{
	unsigned char	m_length[17];		// 17 values from jpg file, 
	// k =1-16 ; L[k] indicates the number of Huffman codes of length k
	unsigned char	m_hufVal[257];		// 256 codes read in from the jpeg file

	int				m_numBlocks;
	struct stBlock			m_blocks[16];
};
struct stComponent 
{
	unsigned int			m_hFactor; // y�� ���ø����
	unsigned int			m_vFactor; // x�� ���ø����
	float *				m_qTable;			// Pointer to the quantisation table to use
	struct stHuffmanTable*		m_acTable; // AC���̺� value
	struct stHuffmanTable2*		m_dcTable; // DC���̺� value
	short int				m_DCT[65];			// DCT���̺� value
	int					m_previousDC;		// ���� DC���(DC����� ���� ��� ����) 
};
struct stJpegData
{
	unsigned char*		m_rgb;				// RGB Pixel Value
	unsigned int		m_width;			// �̹��� �ʺ�
	unsigned int		m_height;			// �̹��� ����

	const unsigned char*m_stream;			// ��Ʈ���� ������?

	struct stComponent			m_component_info[COMPONENTS]; // ������Ʈ ������

	float				m_Q_tables[COMPONENTS][64];	// ����ȭ���̺�
	struct stHuffmanTable2		m_HTDC[HUFFMAN_TABLES];		// ������ DC���̺�  
	struct stHuffmanTable		m_HTAC[HUFFMAN_TABLES];		// ������ AC���̺�

	// Temp space used after the IDCT to store each components
	unsigned char		m_Y[64*4];		// Y�� �ӽ�����
	unsigned char		m_Cr[64];		// Cr�� �ӽ����� 
	unsigned char		m_Cb[64];		// Cb�� �ӽ�����

	// Internal Pointer use for colorspace conversion, do not modify it !!!

	// ��ϴ��� �ּ�
	unsigned char *		m_colourspace; 
};
struct stBMFH // BitmapFileHeader & BitmapInfoHeader
{
	// BitmapFileHeader
	//char         bmtype[2];     // 2 bytes - 'B' 'M'
	unsigned int iFileSize;     // 4 bytes
	short int    reserved1;     // 2 bytes
	short int    reserved2;     // 2 bytes
	unsigned int iOffsetBits;   // 4 bytes
	// End of stBMFH structure - size of 14 bytes
	// BitmapInfoHeader
	unsigned int iSizeHeader;    // 4 bytes - 40
	unsigned int iWidth;         // 4 bytes
	unsigned int iHeight;        // 4 bytes
	short int    iPlanes;        // 2 bytes
	short int    iBitCount;      // 2 bytes
	unsigned int Compression;    // 4 bytes
	unsigned int iSizeImage;     // 4 bytes
	unsigned int iXPelsPerMeter; // 4 bytes
	unsigned int iYPelsPerMeter; // 4 bytes
	unsigned int iClrUsed;       // 4 bytes
	unsigned int iClrImportant;  // 4 bytes
	// End of stBMIF structure - size 40 bytes
	// Total size - 54 bytes
};


// ������

// ���ϻ����� ���ϱ�
int FileSize(FILE *fp)
{
	long pos;
	fseek(fp, 0, SEEK_END);
	pos = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	return pos;
};

unsigned char Clamp(int i)
{
	if (i<0)
		return 0;
	else if (i>255)
		return 255;
	else
		return i;
};


//



// jpg�� bmp��...
int ConvertJpgFile(char* jpgFileName, char * bmpFileName)
{
	FILE *fp;		
	unsigned int lengthOfFile;	// ����ũ��
	unsigned char *buf;			// ����

	unsigned char* rgbpix = NULL;
	unsigned int width  = 0;
	unsigned int height = 0;

	// ���Ͽ���
	fp = fopen(jpgFileName, "rb");
	if (fp == NULL)
	{
		printf("Cannot open jpg file: %s\n", jpgFileName);
		return -1;
	}
	// ���ϻ����� ����
	lengthOfFile = FileSize(fp);

	// ���� �ʱ�ȭ
	buf = (unsigned char*)malloc(sizeof(unsigned char) * lengthOfFile + 4);

	if (buf == NULL)
	{
		printf("Not enough memory for loading file\n");
		return -1;
	}
	// ���ۿ� ������ ����
	fread(buf, lengthOfFile, 1, fp);
	fclose(fp);

	// jpeg ���ڵ�(����, ���ϻ�����, .....)
	DecodeJpgFileData(buf, lengthOfFile, &rgbpix, &width, &height);

	free(buf);

	//------------ ��ϴ��� �ּ�--------------// 
	if (rgbpix==NULL)
	{
		printf("rgbpix is null....Failed to bmp\n");
		return -1;
	}

	WriteBMP24(bmpFileName, width, height, rgbpix);
	
	free(rgbpix);
	
	//---------- ��ϴ��� �ּ� ��------------//

	return 0;
}
// ���ڵ� ����
void DecodeJpgFileData(const unsigned char* buf, const int sizeBuf, unsigned char** rgbpix,	unsigned int* width, unsigned int* height)
{
	// jpeg ����ü �ʱ�ȭ
	struct stJpegData *jdec;

	jdec = (struct stJpegData *)malloc(sizeof(struct stJpegData));

	// jpeg ����Ľ��ؼ� ����ü�� ����(jpeg, ����, ������)
	if (JpegParseHeader(jdec, buf, sizeBuf)<0)
	{
		printf("ERROR > parsing jpg header\n");
		exit(1);
	}

	printf("Decoding JPEG image...\n");
	JpegDecode(jdec);

	//------------ ��ϴ��� �ּ�--------------// 
	
	JpegGetImageSize(jdec, width, height);

	*rgbpix = jdec->m_rgb;
	//---------- ��ϴ��� �ּ� ��------------//
	free(jdec);
}
// �̹����� �ʺ�, ����
void JpegGetImageSize(struct stJpegData *jdata, unsigned int *width, unsigned int *height)
{
	*width  = jdata->m_width;
	*height = jdata->m_height;
}
// SOI parsing
int JpegParseHeader(struct stJpegData *jdata, const unsigned char *buf, unsigned int size)
{
	const unsigned char* startStream; // ��Ʈ��
	const int fileSize = size-2; // ���ϻ�����

	// Start Of Image ��ĿȮ��
	if ((buf[0] != 0xFF) || (buf[1] != SOI))
	{
		printf("Not a JPG file ?\n");
		return -1;
	}

	startStream = buf+2;
	// �Ľ� ����
	if(ParseJFIF(jdata, startStream) > 0)
	{
		printf("-|- File thinks its size is: %d bytes\n", fileSize);
		return 0;
	}
	else
	{
		printf("Not a JPG file ?\n");
		return -1;
	}

}
// Jpeg ���� �Ľ�
int ParseJFIF(struct stJpegData *jdata, const unsigned char *stream)
{
	int chuck_len; // �ش� ��Ŀ�� length
	int marker; // ��Ŀ...FF�� ������ ���� 1����Ʈ
	int sos_marker_found = 0; // SOS ã�Ҵ��� flag
	int dht_marker_found = 0; // DHT ã�Ҵ��� flag

	// ��Ŀ �Ľ�
	while (!sos_marker_found)
	{
		// �� ��Ŀ�� ������ FF�� �ƴϸ� ��Ŀ�� �ƴ�
		if (*stream++ != 0xff)
		{
			goto bogus_jpeg_format;
		}

		// ��Ŀ�� FF�� �����ϸ� ������ 1�������Ѽ� ��Ŀ�� ���� Ȯ��
		while (*stream == 0xff)
		{
			stream++;
		}

		marker = *stream++; // ��Ŀ�� FF ���ڸ��� �����ϰ� ���������ͷ� �̵�

		chuck_len = BYTE_TO_WORD(stream); // ��Ŀ�� length ���

		switch (marker)
		{
		case SOF: // FFC0
			{
				if (ParseSOF(jdata, stream) < 0)
					return -1;
			}
			break;
		case DQT: // FFDB
			{
				if (ParseDQT(jdata, stream) < 0)
					return -1;
			}
			break;
		case SOS: // FFDA
			{
				if (ParseSOS(jdata, stream) < 0)
					goto no_YCBCR_jpeg_format;

				sos_marker_found = 1; // SOS�� ����� �������̱� ������ while ����
			}
			break; 
		case DHT: // FFC4
			{
				if (ParseDHT(jdata, stream) < 0)
					return -1;
				dht_marker_found = 1;
			}
			break;

		case SOI: // FFD8, FFD9 �� �� ��Ŀ�� ��Ŀ�����Ͱ� �����Ƿ� length�� ����
		case EOI:
			{
				chuck_len = 0;
				break;
			}
			break;
		case 0xDD: //DRI: Restart_markers=1;
			{
				printf("DRI - Restart_marker\n");
			}
			break;
		case APP0: // FFE0
			{
				printf("APP0 Chunk ('txt' information) skipping\n");
			}
			break;
		default:
			{
				printf("ERROR> Unknown marker %2.2x\n", marker);
			}
			break;
		}
		stream += chuck_len; // ��Ʈ�� ������ ����(��Ŀ�� length��ŭ)
	}
	if (!dht_marker_found) // DHT ��ã��
	{
		printf("ERROR> No Huffman table loaded\n");
		return -1;
	}
	if(!sos_marker_found)
	{
		printf("ERROR> No SOS data\n");
		return -1;
	}

	return 1;

	// jpeg ������ �ƴѰ��
bogus_jpeg_format:
	printf("ERROR> Bogus jpeg format\n");
	return -1;
no_YCBCR_jpeg_format:
	printf("ERROR> jpeg is no YCBCR\n");
	return -1;
}
int ParseSOF(struct stJpegData *jdata, const unsigned char *stream)
{
	/*
	SOF		16		0xffc0		Start Of Frame
	Lf		16		3Nf+8		Frame header length
	P		8		8			Sample precision
	Y		16		0-65535		Number of lines
	X		16		1-65535		Samples per line
	Nf		8		1-255		Number of image components (e.g. Y, U and V).

	---------Repeats for the number of components (e.g. Nf)-----------------
	Ci		8		0-255		Component identifier
	Hi		4		1-4			Horizontal Sampling Factor
	Vi		4		1-4			Vertical Sampling Factor
	Tqi		8		0-3			Quantization Table Selector.
	*/

	int height; // �̹��� ����, �ʺ�
	int width;
	int nr_components; // �������� [0]:index, [1]:���ø����(horizontal,vertical), [2]:�������̺�id
	int i;

	int rt = SOFPrint(stream);

	height = BYTE_TO_WORD(stream+3);
	width  = BYTE_TO_WORD(stream+5);
	nr_components = stream[7];

	stream += 8;
	for (i=0; i<nr_components; i++) 
	{
		int cid				= *stream++;
		int sampling_factor = *stream++;
		int Q_table			= *stream++;

		// ���� ����ü�� ���� set
		struct stComponent *c = &jdata->m_component_info[cid];
		c->m_vFactor = sampling_factor&0xf;
		c->m_hFactor = sampling_factor>>4;
		c->m_qTable = jdata->m_Q_tables[Q_table];

		printf("����:%d  ���ø����:%dx%d  �������̺� ID:%d\n",
			cid, 
			c->m_vFactor, 
			c->m_hFactor, 
			Q_table );
	}
	jdata->m_width = width;
	jdata->m_height = height;

	return 0;
}
int SOFPrint(const unsigned char *stream)
{
	int width, height; // �̹��� �ʺ�, ����
	int nr_components; // Y, Cb, Cr ���� ��
	int precision; // ���ø� ��Ʈ��

	// ������Ʈ ���� ���� ���...YCbCr�� ���ڵ��� �� �ִ�
	const char *nr_components_to_string[] =	{	"????",
		"Grayscale",
		"????",
		"YCbCr",
		"CYMK" };

	precision = stream[2];
	height = BYTE_TO_WORD(stream+3); // ���̰� �տ� �ִٴ°ſ� ����
	width  = BYTE_TO_WORD(stream+5);
	nr_components = stream[7];

	//printf("> SOF marker\n");
	//printf("�̹���������:%dx%d Y,Cb,Cr���� ��:%d (%s)  ���ø���Ʈ��:%d\n", width, height, nr_components, nr_components_to_string[nr_components], precision);

	return 0;
}
/***************************************************************************/
int ParseDQT(struct stJpegData *jdata, const unsigned char *stream)
{
	int length, qi; // DQT Length, �������̺�ID
	float *table; // �������̺��� �ּ�
	int qprecision; // ���̺�ID ���� 4bit
	int qindex; // ���̺�ID ���� 4bit

	printf("> DQT marker\n");
	length = BYTE_TO_WORD(stream) - 2; // length �� ũ��(2byte)��ŭ ����
	stream += 2;	// length �Ѿ

	// ����ȭ���̺� ������ŭ while���� ����
	while (length>0)
	{
		// ����ȭ���̺��� ù��° ��(�������̺��� id)�� qi�� �ְ� ��Ʈ�� ������ ����
		qi = *stream++;


		qprecision = qi>>4;	 // ���̺�id�� ���� 4��Ʈ
		qindex     = qi&0xf; // ���̺�id�� ���� 4��Ʈ

		if (qprecision)
		{
			// precision in this case is either 0 or 1 and indicates the precision 
			// of the quantized values;
			// 8-bit (baseline) for 0 and  up to 16-bit for 1 

			printf("Error - 16 bits quantization table is not supported\n");
		}

		if (qindex>4)
		{
			printf("Error - No more 4 quantization table is supported (got %d)\n", qi);
		}


		// table�� ���� �������̺��� �ּҷ� �ʱ�ȭ
		table = jdata->m_Q_tables[qindex];

		// �������̺��� table�� set
		BuildQuantizationTable(table, stream);

		stream += 64; // ��Ʈ�� ����
		length -= 65; // length ����(while ����� ����)
	}
	return 0;
}
void BuildQuantizationTable(float *qtable, const unsigned char *ref_table)
{
	int c = 0;
	int i, j;

	for (i=0; i<8; i++) 
	{
		for (j=0; j<8; j++) 
		{
			unsigned char val = ref_table[c];

			qtable[c] = val; // 8*8 ����ȭ ���̺��� ���������� 1�����迭�� set
			c++;
		}
	}
}
/***************************************************************************/

int ParseSOS(struct stJpegData *jdata, const unsigned char *stream)
{
	/*
	SOS		16		0xffd8			Start Of Scan
	Ls		16		2Ns + 6			Scan header length
	Ns		8		1-4				Number of image components
	Csj		8		0-255			Scan Component Selector
	Tdj		4		0-1				DC Coding Table Selector
	Taj		4		0-1				AC Coding Table Selector
	Ss		8		0				Start of spectral selection
	Se		8		63				End of spectral selection
	Ah		4		0				Successive Approximation Bit High
	Ai		4		0				Successive Approximation Bit Low
	*/

	unsigned int nr_components = stream[2]; // ���� ��
	unsigned int i;
	printf("> SOS marker\n");

	if (nr_components != 3) // ������ Y,Cb, Cr �� 3�������Ѵ�
	{
		printf("Error - We only support YCbCr image\n");
		return -1;
	}


	stream += 3; // ��Ʈ�� ������ 3byte ����
	for (i=0;i<nr_components;i++) 
	{
		unsigned int cid   = *stream++; // ����id
		unsigned int table = *stream++; // ���������̺�id (00 , 01, 10, 11)

		if ((table&0xf)>=4)
		{
			printf("Error - We do not support more than 2 AC Huffman table\n");
		}
		if ((table>>4)>=4)
		{
			printf("Error - We do not support more than 2 DC Huffman table\n");
		}
		printf("ComponentId:%d  tableAC:%d tableDC:%d\n", cid, table&0xf, table>>4);

		jdata->m_component_info[cid].m_acTable = &jdata->m_HTAC[table&0xf]; // ���������̺� ����
		jdata->m_component_info[cid].m_dcTable = &jdata->m_HTDC[table>>4]; // ���������̺� ����
	}
	jdata->m_stream = stream+3; // ��Ʈ�������͸� scanData�� ����
	return 0;
}

/***************************************************************************/

int ParseDHT(struct stJpegData *jdata, const unsigned char *stream)
{
	/*
	u8 0xff 
	u8 0xc4 (type of segment) 
	u16 be length of segment 
	4-bits class (0 is DC, 1 is AC, more on this later) 
	4-bits table id 
	array of 16 u8 number of elements for each of 16 depths 
	array of u8 elements, in order of depth 
	*/

	unsigned int count, i;
	unsigned char huff_bits[17]; // 1~16��Ʈ�� ���� �󵵼��� ���� �迭
	int length, index; // length, ���������̺� id(00 : Y-DC, 10 : Y-AC, 01 : CbCr-DC, 11 : CbCr-AC)

	length = BYTE_TO_WORD(stream) - 2; // marker���� FF�� �� �κ�
	stream += 2;	// Skip length

	printf("> DHT marker (length=%d)\n", length);

	while (length>0) 
	{
		index = *stream++;

		huff_bits[0] = 0; // 0��Ʈ�� ���⶧���� 0�� ����

		count = 0; // �� �󵵼��� counter
		for (i=1; i<17; i++) 
		{
			huff_bits[i] = *stream++;
			count += huff_bits[i]; // �ε����� code length, ���� �󵵼�
		}

		// count �� �� �󵵼�
		if (count > 256)
		{
			printf("Error - No more than 1024 bytes is allowed to describe a huffman table");
		}
		if ( (index &0xf) >= 4)
		{
			printf("Error - No mode than %d Huffman tables is supported\n", 4);
		}
		printf("Huffman table %s n%d\n", (index&0xf0)?"AC":"DC", index&0xf);
		printf("Length of the table: %d\n", count);

		// AC�� DC �и�
		// DC : 00, 01
		// AC : 10, 11
		if (index & 0xf0 ) // AC
		{
			// ���������̺��� �����Ͱ��� ��� �迭
			unsigned char* huffval = jdata->m_HTAC[index&0xf].m_hufVal;
			for (i = 0; i < count; i++)
				huffval[i] = *stream++;
			// ���������̺� �����(�󵵼� �迭[0~16], ��Ʈ��, ������AC���̺�[�������̺�id])
			BuildHuffmanTable(huff_bits, stream, &jdata->m_HTAC[index&0xf]);
		}
		else // DC
		{
			unsigned char* huffval = jdata->m_HTDC[index&0xf].m_hufVal; // �ɺ����� ������ m_hufVal �迭�� ������
			for (i = 0; i < count; i++)
				huffval[i] = *stream++; // symbol �� ����
			// ���������̺� �����(�󵵼� �迭[0~16], ��Ʈ��, ������DC���̺�[�������̺�id])
			BuildHuffmanTable(huff_bits, stream, &jdata->m_HTDC[index&0xf]);
		}

		// while���� ����� ����
		length -= 1;
		length -= 16;
		length -= count;
	}
	printf("< DHT marker\n");
	return 0;
}
void BuildHuffmanTable(const unsigned char *bits, const unsigned char *stream, struct stHuffmanTable *HT)
{
	int i, j, k;
	int numBlocks = 0; // ��Ʈ�� �� �󵵼�
	int c=0;

	// ���������̺��� code length�� �󵵼� ����
	for (j=1; j<=16; j++) 
	{
		HT->m_length[j] = bits[j]; // bits �迭�� m_length�� ����(idx : code_length, val : count)
	}
	// ��Ʈ�� �� �󵵼� ���ϱ�
	for (i=1; i<=16; i++)
	{
		numBlocks += HT->m_length[i]; // count�� ����
	}
	HT->m_numBlocks = numBlocks; // �� �󵵼��� HT(���������̺� ����ü)->m_numBlocks�� ����


	for (i=1; i<=16; i++)
	{
		for (j=0; j<HT->m_length[i]; j++)
		{
			HT->m_blocks[c].length = i; // ������ code length���� m_blocks ����ü�� ����
			c++;
		}

	}
	// ���������̺�(HT)�� ���� set �Ѵ�....m_blocks,  m_hufVal
	GenHuffCodes(HT->m_numBlocks, HT->m_blocks, HT->m_hufVal);

	//for(k=0;k<HT->m_numBlocks;k++)
	//{
	//	//printf("codeLength : %d, symbol : 0x%02x, code : %s\n", HT->m_blocks[k].length, HT->m_blocks[k].value, byte_to_binary(HT->m_blocks[k].code));

	//}
}

void GenHuffCodes( int num_codes, struct stBlock* arr, unsigned char* huffVal )
{
	int hufcounter = 0;
	int codelengthcounter = 1;
	int cc;
	// �� �󵵼� ��ŭ
	for(cc=0; cc< num_codes; cc++)
	{
		while ( arr[cc].length > codelengthcounter )
		{
			hufcounter = hufcounter << 1;
			codelengthcounter++;
		}

		arr[cc].code = hufcounter; // ������ code���� m_blocks ����ü�� ����
		arr[cc].value = huffVal[cc]; // ������ symbol���� m_blocks ����ü�� ����
		hufcounter = hufcounter + 1;
	}
}



// Decode
int JpegDecode(struct stJpegData *jdata)
{
	int hFactor = jdata->m_component_info[cY].m_hFactor; // ����
	int vFactor = jdata->m_component_info[cY].m_vFactor; // ����

	int xstride_by_mcu = 8*hFactor; // MCU
	int ystride_by_mcu = 8*vFactor;

	int jpgHeight = (int)jdata->m_height;
	int jpgWidth = (int)jdata->m_width;

	int cnt=0;
	int x, y;

	int h = jdata->m_height*3;
	int w = jdata->m_width*3;
	// 0 padding ������ �� ũ�� �����ɱ�?
	int height = h + (8*hFactor) - (h%(8*hFactor));
	int width  = w + (8*vFactor) - (w%(8*vFactor));
	jdata->m_rgb = (unsigned char *)calloc(width*height, sizeof(unsigned char) );


	jdata->m_component_info[0].m_previousDC = 0;
	jdata->m_component_info[1].m_previousDC = 0;
	jdata->m_component_info[2].m_previousDC = 0;
	jdata->m_component_info[3].m_previousDC = 0;

	// Just the decode the image by 'macroblock' (size is 8x8, 8x16, or 16x16)
	for (y=0 ; y<jpgHeight; y+=ystride_by_mcu)
	{
		for (x=0; x<jpgWidth; x+=xstride_by_mcu)
		{
			cnt++;
			//---------- ��ϴ��� �ּ�------------//
			jdata->m_colourspace = jdata->m_rgb + x*3 + (y *jdata->m_width*3);
			//---------- ��ϴ��� �ּ� ��------------//
			// Decode MCU Plane
			DecodeMCU(jdata, hFactor, vFactor );

			YCrCB_to_RGB24_Block8x8(jdata, hFactor, vFactor, x, y, jdata->m_width, jdata->m_height);
		}
	}
	return 0;
}
void DecodeMCU(struct stJpegData *jdata, int w, int h)
{
	int x, y;
	// Y
	for (y=0; y<h; y++)
	{
		for (x=0; x<w; x++)
		{
			int stride = w*8;
			int offset = x*8 + y*64*w;
			//printf("------------------Y------------------\n");
			ProcessHuffmanDataUnit(jdata, cY); // ���������ڵ� 

			DecodeSingleBlock(&jdata->m_component_info[cY], &jdata->m_Y[offset], stride);
			//printf("--------------------------------------\n");
		}
	}

	// Cb
	//printf("-----------------Cb-----------------\n");
	ProcessHuffmanDataUnit(jdata, cCb);
	DecodeSingleBlock(&jdata->m_component_info[cCb], jdata->m_Cb, 8);
	//printf("--------------------------------------\n");
	// Cr
	//printf("-----------------Cr-----------------\n");
	ProcessHuffmanDataUnit(jdata, cCr);
	DecodeSingleBlock(&jdata->m_component_info[cCr], jdata->m_Cr, 8);
	//printf("--------------------------------------\n");
}
void YCrCB_to_RGB24_Block8x8(struct stJpegData *jdata, int w, int h, int imgx, int imgy, int imgw, int imgh)
{
	
	const unsigned char *Y, *Cb, *Cr;

	//---------- ��ϴ��� �ּ� ------------//
	unsigned char *pix;
	//---------- ��ϴ��� �ּ� ��------------//

	unsigned char blockData[768];


	int r, g, b, x, y, cnt;
	int olw, olh;

	cnt = 0;

	Y  = jdata->m_Y;
	Cb = jdata->m_Cb;
	Cr = jdata->m_Cr;

	olw = 0; // overlap
	if ( imgx > (imgw-8*w) )
	{
		olw = imgw-imgx;
	}

	olh = 0; // overlap
	if ( imgy > (imgh-8*h) )
	{
		olh = imgh-imgy;
	}

	//printf("--------------------------------------pixel-------------------------------------\n");
	for (y=0; y<(8*h - olh); y++)
	{
		for (x=0; x<(8*w - olw); x++)
		{
			//---------- ��ϴ��� �ּ� ------------//
			int poff = (x * 3) + (jdata->m_width * 3 * y);
			
			//---------- ��ϴ��� �ּ� ��------------//
			int yoff = x + y * (w * 8);
			int coff = (int)((1.0f/w)*x) + (int)((1.0f/h)*y) * 8;

			int yc =  Y[yoff];
			int cb = Cb[coff];
			int cr = Cr[coff];


			ConvertYCrCbtoRGB(yc,cr,cb,&r,&g,&b);
			//---------- ��ϴ��� �ּ� ------------//

			pix = &(jdata->m_colourspace[poff]);

			pix[0] = Clamp(r);
			pix[1] = Clamp(g);
			pix[2] = Clamp(b);
			//---------- ��ϴ��� �ּ� ��------------//

			blockData[cnt] = Clamp(r);
			blockData[cnt+1] = Clamp(g);
			blockData[cnt+2] = Clamp(b);

			cnt += 3;
		}
	}

	DrawBlockData(blockData, (8*w - olw) , (8*w - olh), imgx, imgy); 


}
void DrawBlockData(unsigned char *buffer, int mcu_x, int mcu_y, int x, int y)
{
	int cnt = 0;
	int i=0;
	int j=0;
	for(j=0;j<mcu_y;j++)
	{
		for(i=0;i<mcu_x;i++)
		{
			// ������ Red��, Green��, Blue������ ���� ������
			//Color rColor = Color::FromArgb(buffer[cnt], buffer[cnt+1], buffer[cnt+2]);

			// �ȼ�����... �� �Ʒ��� ���� ����
			//bmp->SetPixel(i + x, j + y, rColor);

			cnt += 3;
		}
	}

}

void ConvertYCrCbtoRGB(int y, int cb, int cr, int* r, int* g, int* b)
{
	float red, green, blue;

	red   = y + 1.402f*(cb-128);
	green = y-0.34414f*(cr-128)-0.71414f*(cb-128);
	blue  = y+1.772f*(cr-128);

	*r = (int) Clamp((int)red);
	*g = (int) Clamp((int)green);
	*b = (int) Clamp((int)blue);
}

void ProcessHuffmanDataUnit(struct stJpegData *jdata, int indx)
{
	struct stComponent *c = &jdata->m_component_info[indx];

	// Start Huffman decoding

	// We memset it here, as later on we can just skip along, when we have lots
	// of leading zeros, for our AC run length encoding :)
	short DCT_tcoeff[64];


	// DC
	int found = 0;
	int decodedValue = 0;
	int k;
	int numDataBits;

	// AC
	int nr=1; 
	int EOB_found=0;
	short data;

	int j;

	memset(DCT_tcoeff, 0, sizeof(DCT_tcoeff)); //Initialize DCT_tcoeff
	//	DumpHufCodes(c->m_dcTable);
	//	DumpHufCodes(c->m_acTable);

	//printf("\nHuff Block:\n\n");


	// First thing is get the 1 DC coefficient at the start of our 64 element
	// block
	for (k=1; k<16; k++)
	{
		// Keep grabbing one bit at a time till we find one thats a huffman code
		int code = LookNBits(&jdata->m_stream, k);

		// Check if its one of our huffman codes
		if (IsInHuffmanCodes(code, k,  c->m_dcTable->m_numBlocks, c->m_dcTable->m_blocks, &decodedValue))
		{
			// Skip over the rest of the bits now.
			SkipNBits(&jdata->m_stream, k);

			found = 1;

			// The decoded value is the number of bits we have to read in next
			numDataBits = decodedValue;

			// We know the next k bits are for the actual data
			if (numDataBits==0)
			{
				DCT_tcoeff[0] = c->m_previousDC;
			}
			else
			{

				short data = GetNBits(&jdata->m_stream, numDataBits);

				// ��Ʈ���� 2�Ǻ����� �ٲ� �� 10������ ��ȯ 
				data = DetermineSign(data, numDataBits);

				DCT_tcoeff[0] = data + c->m_previousDC;
				c->m_previousDC = DCT_tcoeff[0];
			}

			// Found so we can exit out
			break;
		}
	}

	if (!found)
	{
		printf("-|- ##ERROR## We have a *serious* error, unable to find huffman code\n");
	}

	// Second, the 63 AC coefficient
	while ( (nr<=63)&&(!EOB_found) )
	{
		int k = 0;
		for (k=1; k<=16; k++)
		{
			// Keep grabbing one bit at a time till we find one thats a huffman code
			int code = LookNBits(&jdata->m_stream, k);


			// Check if its one of our huffman codes
			if (IsInHuffmanCodes(code, k,  c->m_acTable->m_numBlocks, c->m_acTable->m_blocks, &decodedValue))
			{
				// Our decoded value is broken down into 2 parts, repeating RLE, and then
				// the number of bits that make up the actual value next
				int valCode = decodedValue;

				unsigned char size_val = valCode&0xF;	// Number of bits for our data
				unsigned char count_0  = valCode>>4;	// Number RunLengthZeros


				// Skip over k bits, since we found the huffman value
				SkipNBits(&jdata->m_stream, k);

				if (size_val==0) 
				{// RLE 
					if (count_0==0)
						EOB_found=1;	// EOB found, go out
					else if (count_0==0xF) 
						nr+=16;  // skip 16 zeros
				}
				else
				{
					nr+=count_0; //skip count_0 zeroes

					if (nr > 63)
					{
						printf("-|- ##ERROR## Huffman Decoding\n");
					}

					data = GetNBits(&jdata->m_stream, size_val );

					data = DetermineSign(data, size_val);

					DCT_tcoeff[nr++]=data;

				}
				break;
			}
		}

		if (k>16)
		{	
			nr++;
		}
	}
	// We've decoded a block of data, so copy it across to our buffer
	for (j = 0; j < 64; j++)
	{
		c->m_DCT[j] = DCT_tcoeff[j];
	}

}
int IsInHuffmanCodes(int code, int numCodeBits, int numBlocks, struct stBlock* blocks, int* outValue)
{
	int j;
	for (j=0; j<numBlocks; j++)
	{
		int hufhCode		= blocks[j].code;
		int hufCodeLenBits	= blocks[j].length;
		int hufValue		= blocks[j].value;

		// We've got a match!
		if ((code==hufhCode) && (numCodeBits==hufCodeLenBits))
		{
			*outValue = hufValue;
			return 1;
		}
	}
	return 0;
}


void DecodeSingleBlock(struct stComponent *comp, unsigned char *outputBuf, int stride)
{
	short* inptr; // �����Ͱ�
	float* quantptr; // ������ȭ��� ��
	unsigned char *outptr;

	// Create a temp 8x8, i.e. 64 array for the data
	int data[64] = {0}; // �ӽ� �����ͺ��(������ȭ)

	int i, x, y;
	int block[64] = {0}; // ��������� ��ģ ���� �����ͺ��
	int arrayBlock[8][8] = {0}; // 2�����迭�� ��ȯ�� �����ͺ��
	int val[8][8]={0}; // 0~255������ ������ �ϴ� �����ͺ��



	inptr    = comp->m_DCT; // DCT �����ͺ��
	quantptr = comp->m_qTable;

	for (i=0; i<64; i++)
	{
		data[i] = inptr[i]; // �ӽ� �����ͺ������ ����
	}

	DequantizeBlock(data, quantptr); // ������ȭ

	DeZigZag(block, data); // ��������׽�ĵ

	TransformArray(arrayBlock, block); // 2�����迭�� ��ȯ(8x8)

	PerformIDCT(val, arrayBlock); // ��DCT ��ȯ

	// output
	outptr = outputBuf;
	for (y = 0; y < 8; y++) 
	{
		for (x=0; x<8; x++)
		{
			// DCT ����� 128�� ����
			val[x][y] += 128;
			// param�� range�� 0~255 �� �����
			outptr[x] = Clamp(val[x][y]);
		}

		outptr += stride;
	}

	//DumpDecodedBlock(val); // val ���
}


void DumpDCTValues(short dct[64])
{
	int i,c = 0;
	printf("\n#Extracted DCT values from SOS#\n");
	for (i=0; i<64; i++)
	{
		printf("% 4d  ", dct[c++]);

		if ( (c>0) && (c%8==0) ) printf("\n");
	}
	printf("\n");
}


int DetermineSign(int val, int nBits)
{
	int negative = val < (1<<(nBits-1));

	if (negative)
	{
		// (-1 << (s)), makes the last bit a 1, so we have 1000,0000 for example for 8 bits

		val = val + (-1 << (nBits)) + 1; 
	}

	// Else its unsigned, just return
	return val;
}





// ��DCT ��ȯ
void PerformIDCT(int outBlock[8][8], const int inBlock[8][8])
{
	int x, y;
	for(y=0; y<8; y++)
	{
		for(x=0; x<8; x++)
		{
			outBlock[x][y]  =  IDCTCalculate( x, y, inBlock); // ��DCT ���
		}
	}
}
// 1�����迭�� 2�����迭�� ��ȯ
void TransformArray(int outArray[8][8], const int inArray[64])
{
	int x, y;
	int cc = 0;
	for(y=0; y<8; y++)
	{
		for(x=0; x<8; x++)
		{
			outArray[x][y]  =  inArray[cc];
			cc++;
		}
	}
}
// ��� print
void DumpDecodedBlock(int val[8][8])
{
	int x, y;
	//printf("# Decoded 8x8 Block#\n");
	for(y=0; y<8; y++)
	{
		for(x=0; x<8; x++)
		{
			//printf("%2x ", val[x][y]);
		}
		//printf("\n");
	}
}
// ������ȭ
void DequantizeBlock( int block[64], const float quantBlock[64] )
{
	int c;
	for(c=0; c<64; c++)
	{
		block[c] = (int)(block[c] * quantBlock[c]); // ������ȭ(���ڰ���� ���Ѵ�)
	}
}
// ������� ����ȯ
void DeZigZag(int outBlock[64], const int inBlock[64])
{
	int i;
	for(i=0; i<64; i++)
	{
		outBlock[ i ] = inBlock[ZigZagArray[i]]; // ������� ����ȯ
	}
}
// IDCT ���
int IDCTCalculate(int x, int y, const int block[8][8])
{
	int ret = 0;
	const float PI = 3.14f;
	float sum=0;
	int u, v;
	for(u=0; u<8; u++)
	{
		for(v=0; v<8; v++)
		{
			float cosf1 = cosf(((2*x+1) * u * PI) / 16);
			float cosf2 = cosf( ((2*y+1) * v * PI) / 16);

			double reu = IdctSum(u);
			double rev = IdctSum(v);
			double d = reu * rev * block[u][v];

			sum += d*cosf1*cosf2;
		}
	}     
	ret = (1.0/4.0) * sum;
	return ret;
}





unsigned int g_reservoir = 0;
unsigned int g_nbits_in_reservoir = 0;

void FillNBits(const unsigned char** stream, int nbits_wanted)
{
	while ((int)g_nbits_in_reservoir<nbits_wanted)
	{
		const unsigned char c = *(*stream)++;
		g_reservoir <<= 8;
		if (c == 0xff && (**stream) == 0x00)
			(*stream)++;
		g_reservoir |= c;
		g_nbits_in_reservoir+=8;
	}
}
short GetNBits(const unsigned char** stream, int nbits_wanted)
{
	short result;

	FillNBits(stream, nbits_wanted);

	result = ((g_reservoir)>>(g_nbits_in_reservoir-(nbits_wanted))); 

	g_nbits_in_reservoir -= (nbits_wanted); 
	g_reservoir &= ((1U<<g_nbits_in_reservoir)-1);

	/*
	// Could do the sign conversion here!
	if (result < (short)(1UL<<((nbits_wanted)-1)))
	{
	result = result + (short)(0xFFFFFFFFUL<<(nbits_wanted))+1;
	}
	*/
	return result;
}
int LookNBits(const unsigned char** stream, int nbits_wanted)
{
	int result;
	FillNBits(stream, nbits_wanted);

	result = ((g_reservoir)>>(g_nbits_in_reservoir-(nbits_wanted)));
	return result;
} 
void SkipNBits(const unsigned char** stream, int nbits_wanted)
{
	FillNBits(stream, nbits_wanted);

	g_nbits_in_reservoir -= (nbits_wanted); 
	g_reservoir &= ((1U<<g_nbits_in_reservoir)-1);
}

void WriteBMP24(const char* szBmpFileName, int Width, int Height, unsigned char* RGB)
{
	struct stBMFH bh;

	FILE *fp;
	int x, y;

	char bfType[2] = {'B', 'M'};

	int iNumPaddedBytes = 4 - (Width * 3) % 4; // 00 padding
	iNumPaddedBytes = iNumPaddedBytes % 4;

	memset(&bh, 0, sizeof(bh));
	//bh.bmtype[0]='B';
	//bh.bmtype[1]='M';


	fp = fopen(szBmpFileName, "wb");


	fwrite(bfType, 2, 1, fp);	// ��Ʈ��id(BM : 42 4D) write 

	// header(52 + 2) + RGB * 3
	bh.iFileSize = (Width*Height*3) + (Height*iNumPaddedBytes) + sizeof(bh) + 2; // ���ϻ�����


	bh.iOffsetBits = sizeof(bh) + 2; // header(52 + 2)
	bh.iSizeHeader = 40; // Image Header
	bh.iPlanes = 1; // plane 1
	bh.iWidth = Width; 
	bh.iHeight = Height;
	bh.iBitCount = 24; // 24��Ʈ bitmap
	bh.iSizeImage = (Width*Height*3) + (Height*iNumPaddedBytes);
	fwrite(&bh, sizeof(bh), 1, fp); // bmp��� write

	//printf("\n\n\n----------------RGB Data---------------\n\n");
	for (y=Height-1; y>=0; y--)
	{
		for (x=0; x<Width; x++)
		{
			int i = (x + (Width)*y) * 3;

			unsigned int rgbpix = (RGB[i]<<16)|(RGB[i+1]<<8)|(RGB[i+2]<<0);
			fwrite(&rgbpix, 3, 1, fp);

			//if(x % 10 ==0)
			//printf("\n");

			//printf("0x%06x ",  rgbpix);

		}
		if (iNumPaddedBytes>0)
		{
			unsigned char pad = 0;
			fwrite(&pad, iNumPaddedBytes, 1, fp);

			//printf("%x ", pad);
		}
		//printf("\n");

	}
	fclose(fp);

	printf("image create");
}