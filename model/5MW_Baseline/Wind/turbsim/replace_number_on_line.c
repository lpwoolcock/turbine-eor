#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include "mex.h"
#include <ctype.h>


#define MAX_NUM_LENGTH 16
#define MAX_LINE_LENGTH 512


/*
update the file with the new line
*/
void update_file(FILE* fp,FILE* new_fp,char* new_line,int line_num){
	char tmp[MAX_LINE_LENGTH]; 
	for(int i=0;i<line_num;i++){
		fputs(fgets(tmp,MAX_LINE_LENGTH,fp),new_fp);
	}
	fputs(new_line,new_fp);
	fgets(tmp,MAX_LINE_LENGTH,fp); //Consume replaced line

	while(fgets(tmp,MAX_LINE_LENGTH,fp)!=NULL){
		fputs(tmp,new_fp);
	}
}


int replace_number_on_line(int line_number,double new_number,char* file_path){
	FILE* fp;
	if((fp = fopen(file_path,"r"))==NULL){
		printf("Error: could not open the file");
		return 1;
	}
	char old_line[MAX_LINE_LENGTH];
	for(int i=0;i<=line_number;i++){

		fgets(old_line,MAX_LINE_LENGTH,fp);
	}
	fclose(fp);

	int index=0;
	while(index<MAX_LINE_LENGTH){
		if(isdigit(old_line[index])){
			break;
		}
		if(old_line[index]=='\0'){
			printf("Error: end of string reached with no number found");
			fclose(fp);
			return 1;
		}
		index++;
	}
	if(index==MAX_LINE_LENGTH){
		printf("Error: end of string reached with no number found");
		fclose(fp);
		return 1;
	}
	
	while(index<MAX_LINE_LENGTH){
		if(isspace(old_line[index])){
			break;
		}
		index++;

	}

	char new_line[MAX_LINE_LENGTH];
	char new_number_string[MAX_NUM_LENGTH];
	sprintf(new_number_string, "%.2lf", new_number);
	int ind_new_line=0;
	while(new_number_string[ind_new_line]!='\0'){
		new_line[ind_new_line] = new_number_string[ind_new_line];
		ind_new_line++;
	}
	while(old_line[index]!='\0'){
		new_line[ind_new_line] = old_line[index];
		ind_new_line++;
		index++;
	}
	new_line[ind_new_line++] = '\0';
	
	if((fp = fopen(file_path,"r"))==NULL){
		printf("Error could not open file");
		return -1;
	}
	FILE* fp_new;
	if((fp_new = fopen("tmp.txt","w"))==NULL){
		printf("Error could not open file");
		fclose(fp);
		return 1;	
	}
	update_file(fp,fp_new,new_line,line_number);

	fclose(fp);
	fclose(fp_new);

	if(remove(file_path)){
		printf("file not removed succesfully\n");
	}

	if(rename("tmp.txt",file_path)){
		printf("error file not renamed\n");
	}

	return 0;
}
/*
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]){
	 if(nrhs!=3) 
      		mexErrMsgIdAndTxt( "MATLAB:revord:invalidNumInputs",
            "One input required.");
     if(replace_number_on_line(atoi(mxArrayToString(prhs[0])),atof(mxArrayToString(prhs[1])),mxArrayToString(prhs[2]))){
     	plhs[0] = mxCreateString("error");
     }
     else{
     	plhs[0] = mxCreateString("success");
     }
     
}*/


int main(int argc, char* argv[]){

	int line_number = atoi(argv[1]);
	int new_number = atoi(argv[2]);
	char* file_path = argv[3];
	replace_number_on_line(line_number,new_number,file_path);


}