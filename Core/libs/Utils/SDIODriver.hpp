/*
 * SDIODriver.hpp
 *
 *  Created on: Apr 27, 2023
 *      Author: DDarie
 */

#ifndef SDIODRIVER_H_
#define SDIODRIVER_H_

#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "fatfs.h"

class SDIODriver
{
public:
	/* mounts the sd card*/
	void Mount_SD (const TCHAR* path);

	/* unmounts the sd card*/
	void Unmount_SD (const TCHAR* path);

	/* Start node to be scanned (***also used as work area***) */
	FRESULT Scan_SD (char* pat);

	/* Only supports removing files from home directory. Directory remover to be added soon */
	FRESULT Format_SD (void);

	/* write the data to the file
	 * @ name : is the path to the file*/
	FRESULT Write_File (char *name, char *data);

	/* read data from the file
	 * @ name : is the path to the file*/
	FRESULT Read_File (char *name);

	/* creates the file, if it does not exists
	 * @ name : is the path to the file*/
	FRESULT Create_File (char *name);

	/* Removes the file from the sd card
	 * @ name : is the path to the file*/
	FRESULT Remove_File (char *name);

	/* creates a directory
	 * @ name: is the path to the directory
	 */
	FRESULT Create_Dir (char *name);

	/* checks the free space in the sd card*/
	void Check_SD_Space (void);

	/* updates the file. write pointer is set to the end of the file
	 * @ name : is the path to the file
	 */
	FRESULT Update_File (char *name, char *data);
};

#endif /* SDIODRIVER_H_ */
