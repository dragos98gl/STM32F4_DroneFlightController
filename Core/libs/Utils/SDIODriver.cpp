/*
 * SDIODriver.cpp
 *
 *  Created on: Apr 27, 2023
 *      Author: DDarie
 */

#include <cstdlib>
#include "SDIODriver.hpp"
#include "stm32f4xx_hal.h"

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

void SDIODriver::Mount_SD (const TCHAR* path)
{
	fresult = f_mount(&fs, path, 1);
	//if (fresult != FR_OK) Send_Uart ("ERROR!!! in mounting SD CARD...\n\n");
	//else Send_Uart("SD CARD mounted successfully...\n");
}

void SDIODriver::Unmount_SD (const TCHAR* path)
{
	fresult = f_mount(NULL, path, 1);
	//if (fresult == FR_OK) Send_Uart ("SD CARD UNMOUNTED successfully...\n\n\n");
	//else Send_Uart("ERROR!!! in UNMOUNTING SD CARD\n\n\n");
}

/* Start node to be scanned (***also used as work area***) */
FRESULT SDIODriver::Scan_SD (char* pat)
{
    DIR dir;
    UINT i;
    char *path = (char*)malloc(20*sizeof (char));
    sprintf (path, "%s",pat);

    fresult = f_opendir(&dir, path);                       /* Open the directory */
    if (fresult == FR_OK)
    {
        for (;;)
        {
            fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR)     /* It is a directory */
            {
            	if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
            	//char *buf = (char*)malloc(30*sizeof(char));
            	//sprintf (buf, "Dir: %s\r\n", fno.fname);
            	//Send_Uart(buf);
            	//free(buf);
                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                fresult = Scan_SD(path);                     /* Enter the directory */
                if (fresult != FR_OK) break;
                path[i] = 0;
            }
            else
            {   /* It is a file. */
           	   //char *buf = (char*)malloc(30*sizeof(char));
               //sprintf(buf,"File: %s/%s\n", path, fno.fname);
               //Send_Uart(buf);
               //free(buf);
            }
        }
        f_closedir(&dir);
    }
    free(path);
    return fresult;
}

/* Only supports removing files from home directory */
FRESULT SDIODriver::Format_SD (void)
{
    DIR dir;
    char *path = (char*)malloc(20*sizeof (char));
    sprintf (path, "%s","/");

    fresult = f_opendir(&dir, path);                       /* Open the directory */
    if (fresult == FR_OK)
    {
        for (;;)
        {
            fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR)     /* It is a directory */
            {
            	if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
            	fresult = f_unlink(fno.fname);
            	if (fresult == FR_DENIED) continue;
            }
            else
            {   /* It is a file. */
               fresult = f_unlink(fno.fname);
            }
        }
        f_closedir(&dir);
    }
    free(path);
    return fresult;
}




FRESULT SDIODriver::Write_File (char *name, char *data)
{

	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		//char *buf = (char*)malloc(100*sizeof(char));
		//sprintf (buf, "ERROR!!! *%s* does not exists\n\n", name);
		//Send_Uart (buf);
	    //free(buf);
	    //return fresult;
	}

	else
	{
	    /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	    	//char *buf = (char*)malloc(100*sizeof(char));
	    	//sprintf (buf, "ERROR!!! No. %d in opening file *%s*\n\n", fresult, name);
	    	//Send_Uart(buf);
	        //free(buf);
	        //return fresult;
	    }

	    else
	    {
	    	fresult = f_write(&fil, data, strlen(data), &bw);
	    	if (fresult != FR_OK)
	    	{
	    		//char *buf = (char*)malloc(100*sizeof(char));
	    		//sprintf (buf, "ERROR!!! No. %d while writing to the FILE *%s*\n\n", fresult, name);
	    		//Send_Uart(buf);
	    		//free(buf);
	    	}

	    	/* Close file */
	    	fresult = f_close(&fil);
	    	if (fresult != FR_OK)
	    	{
	    		//char *buf = (char*)malloc(100*sizeof(char));
	    		//sprintf (buf, "ERROR!!! No. %d in closing file *%s* after writing it\n\n", fresult, name);
	    		//Send_Uart(buf);
	    		//free(buf);
	    	}
	    	else
	    	{
	    		//char *buf = (char*)malloc(100*sizeof(char));
	    		//sprintf (buf, "File *%s* is WRITTEN and CLOSED successfully\n", name);
	    		//Send_Uart(buf);
	    		//free(buf);
	    	}
	    }
	    return fresult;
	}
}

FRESULT SDIODriver::Read_File (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		//char *buf = (char*)malloc(100*sizeof(char));
		//sprintf (buf, "ERRROR!!! *%s* does not exists\n\n", name);
		//Send_Uart (buf);
		//free(buf);
	    //return fresult;
	}

	else
	{
		/* Open file to read */
		fresult = f_open(&fil, name, FA_READ);

		if (fresult != FR_OK)
		{
			//char *buf = (char*)malloc(100*sizeof(char));
			//sprintf (buf, "ERROR!!! No. %d in opening file *%s*\n\n", fresult, name);
		    //Send_Uart(buf);
		    //free(buf);
			//return fresult;
		}

		/* Read data from the file
		* see the function details for the arguments */

		char *buffer = (char*)malloc(sizeof(f_size(&fil)));
		fresult = f_read (&fil, buffer, f_size(&fil), &br);
		if (fresult != FR_OK)
		{
			//char *buf = (char*)malloc(100*sizeof(char));
			//free(buffer);
		 	//sprintf (buf, "ERROR!!! No. %d in reading file *%s*\n\n", fresult, name);
		  	//Send_Uart(buffer);
		  	//free(buf);
		}

		else
		{
			//Send_Uart(buffer);
			//free(buffer);

			/* Close file */
			fresult = f_close(&fil);
			if (fresult != FR_OK)
			{
				//char *buf = (char*)malloc(100*sizeof(char));
				//sprintf (buf, "ERROR!!! No. %d in closing file *%s*\n\n", fresult, name);
				//Send_Uart(buf);
				//free(buf);
			}
			else
			{
				//char *buf = (char*)malloc(100*sizeof(char));
				//sprintf (buf, "File *%s* CLOSED successfully\n", name);
				//Send_Uart(buf);
				//free(buf);
			}
		}
	    return fresult;
	}
}

FRESULT SDIODriver::Create_File (char *name)
{
	fresult = f_stat (name, &fno);
	if (fresult == FR_OK)
	{
		//char *buf = (char*)malloc(100*sizeof(char));
		//sprintf (buf, "ERROR!!! *%s* already exists!!!!\n use Update_File \n\n",name);
		//Send_Uart(buf);
		//free(buf);
	    //return fresult;
	}
	else
	{
		fresult = f_open(&fil, name, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);
		if (fresult != FR_OK)
		{
			//char *buf = (char*)malloc(100*sizeof(char));
			//sprintf (buf, "ERROR!!! No. %d in creating file *%s*\n\n", fresult, name);
			//Send_Uart(buf);
			//free(buf);
		    //return fresult;
		}
		else
		{
			//char *buf = (char*)malloc(100*sizeof(char));
			//sprintf (buf, "*%s* created successfully\n Now use Write_File to write data\n",name);
			//Send_Uart(buf);
			//free(buf);
		}

		fresult = f_close(&fil);
		if (fresult != FR_OK)
		{
			//char *buf = (char*)malloc(100*sizeof(char));
			//sprintf (buf, "ERROR No. %d in closing file *%s*\n\n", fresult, name);
			//Send_Uart(buf);
			//free(buf);
		}
		else
		{
			//char *buf = (char*)malloc(100*sizeof(char));
			//sprintf (buf, "File *%s* CLOSED successfully\n", name);
			//Send_Uart(buf);
			//free(buf);
		}
	}
    return fresult;
}

FRESULT SDIODriver::Update_File (char *name, char *data)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		//char *buf = (char*)malloc(100*sizeof(char));
		//sprintf (buf, "ERROR!!! *%s* does not exists\n\n", name);
		//Send_Uart (buf);
		//free(buf);
	    //return fresult;
	}

	else
	{
		 /* Create a file with read write access and open it */
	    fresult = f_open(&fil, name, FA_OPEN_APPEND | FA_WRITE);
	    if (fresult != FR_OK)
	    {
	    	//char *buf = (char*)malloc(100*sizeof(char));
	    	//sprintf (buf, "ERROR!!! No. %d in opening file *%s*\n\n", fresult, name);
	    	//Send_Uart(buf);
	        //free(buf);
	        //return fresult;
	    }

	    /* Writing text */
	    fresult = f_write(&fil, data, strlen (data), &bw);
	    if (fresult != FR_OK)
	    {
	    	//char *buf = (char*)malloc(100*sizeof(char));
	    	//sprintf (buf, "ERROR!!! No. %d in writing file *%s*\n\n", fresult, name);
	    	//Send_Uart(buf);
	    	//free(buf);
	    }

	    else
	    {
	    	//char *buf = (char*)malloc(100*sizeof(char));
	    	//sprintf (buf, "*%s* UPDATED successfully\n", name);
	    	//Send_Uart(buf);
	    	//free(buf);
	    }

	    /* Close file */
	    fresult = f_close(&fil);
	    if (fresult != FR_OK)
	    {
	    	//char *buf = (char*)malloc(100*sizeof(char));
	    	//sprintf (buf, "ERROR!!! No. %d in closing file *%s*\n\n", fresult, name);
	    	//Send_Uart(buf);
	    	//free(buf);
	    }
	    else
	    {
	    	//char *buf = (char*)malloc(100*sizeof(char));
	    	//sprintf (buf, "File *%s* CLOSED successfully\n", name);
	    	//Send_Uart(buf);
	    	//free(buf);
	     }
	}
    return fresult;
}

FRESULT SDIODriver::Remove_File (char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat (name, &fno);
	if (fresult != FR_OK)
	{
		//char *buf = (char*)malloc(100*sizeof(char));
		//sprintf (buf, "ERROR!!! *%s* does not exists\n\n", name);
		//Send_Uart (buf);
		//free(buf);
		//return fresult;
	}
	else
	{
		fresult = f_unlink (name);
		if (fresult == FR_OK)
		{
			//char *buf = (char*)malloc(100*sizeof(char));
			//sprintf (buf, "*%s* has been removed successfully\n", name);
			//Send_Uart (buf);
			//free(buf);
		}

		else
		{
			//char *buf = (char*)malloc(100*sizeof(char));
			//sprintf (buf, "ERROR No. %d in removing *%s*\n\n",fresult, name);
			//Send_Uart (buf);
			//free(buf);
		}
	}
	return fresult;
}

FRESULT SDIODriver::Create_Dir (char *name)
{
    fresult = f_mkdir(name);
    if (fresult == FR_OK)
    {
    	//char *buf = (char*)malloc(100*sizeof(char));
    	//sprintf (buf, "*%s* has been created successfully\n", name);
    	//Send_Uart (buf);
    	//free(buf);
    }
    else
    {
    	//char *buf = (char*)malloc(100*sizeof(char));
    	//sprintf (buf, "ERROR No. %d in creating directory *%s*\n\n", fresult,name);
    	//Send_Uart(buf);
    	//free(buf);
    }
    return fresult;
}

void SDIODriver::Check_SD_Space (void)
{
    /* Check free space */
    f_getfree("", &fre_clust, &pfs);

    total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
}

