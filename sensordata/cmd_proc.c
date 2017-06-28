/*
 * cmd_proc.c
 *
 *  Created on: Aug 5, 2010
 *      Author: root
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "global_def.h"

void usage(char *temp);

void cmd_proc(char *file_name, int argc, char *argv[])
{
    extern char *optarg;
    int optchar;

	  if(argc==1) {
	    usage(argv[0]);
	  }

	  while((optchar = getopt(argc, argv, "abc:g:e")) != EOF) {
	    switch((char)optchar) {
	    case 'g':
	      /** Global Parameters File **/
	      strcpy(file_name, optarg);
	      break;
	    default:
	      /** Print Usage if Invalid Command Line Arguments **/
	      usage(argv[0]);
	      break;
	    }
	  }

	}

void usage(char *temp)
{
  fprintf(stderr,"Usage: %s -g<model_control_file>\n",temp);
  fprintf(stderr,"\t<model_control_file> is a file that contains all needed model\n\t\tparameters. If not defined, default values will be used.\n");
}
