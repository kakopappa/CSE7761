
extern void cse7761_SPI_init(void);
extern void cse7761_SPI_Read(unsigned char vaddr,unsigned char *vpp,unsigned char vsize);
extern void cse7761_SPI_WriteRegister(void);
extern void delay2uS(void);
extern unsigned char vHaveData;
int CSE7761_Read_IA(void);
int CSE7761_Read_U(void);
int CSE7761_Read_PA(void);
int CSE7761_Read_EA(void);
