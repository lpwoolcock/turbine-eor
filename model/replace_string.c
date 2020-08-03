#include <stdio.h>
#include <string.h>
#include "mex.h"


#define MAX_STRING_LENGTH 256
#define MAX_LINE_LENGTH 512



/*
	WARNING: risk of buffer overflow
*/
void replace_string(char* new_line, char* from_string, char* to_string, char* tmp,int dif){
	int str_itr=0;
	while(dif--){
		new_line[str_itr++] = *tmp;
		tmp++;
	}
	
	tmp+=strlen(from_string);

	while(*to_string!='\0'){
		new_line[str_itr++] = *to_string;
		to_string++;
	}
	while(*tmp!='\0'){
		new_line[str_itr++] = *tmp;
		tmp++;
	}
	new_line[str_itr] = '\0';
	
}

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

int find_replace_string(char* from_string,char* to_string,char* file_path){
	FILE* fp; 
	if((fp = fopen(file_path,"r")) ==NULL){
		printf("Input File Not Found\n");
		printf("Usage: ./replace_string.exe \"string to change\" \"new string\" \"file path\" ");
		return -1;
	}
	FILE* new_fp;
	if((new_fp = fopen("tmp.txt","w"))==NULL){
		printf("New File Not Created\n");
		return -1;
	}

	char tmp[MAX_LINE_LENGTH];
	int line_num = 0;
	char* loc;
	int flag = 0;
	while(fgets(tmp,MAX_LINE_LENGTH,fp)!=NULL){
		 if((loc = strstr(tmp,from_string)) !=NULL){
		 	flag = 1;
		 	break;
		 }
		line_num++;
	}
	if(!flag){
		printf("Error string not found\n");
		printf("Usage: ./replace_string.exe \"string to change\" \"new string\" \"file path\" ");
		fclose(fp);
		fclose(new_fp);
		return -1;
	}

	fclose(fp);
	
	char new_line[MAX_LINE_LENGTH];
	int dif = loc-tmp;
	replace_string(new_line,from_string,to_string,tmp,dif);
	
	if((fp = fopen(file_path,"r"))==NULL){
		printf("Error could not re-open file\n");
		return -1;
	}
	update_file(fp,new_fp,new_line,line_num);

	fclose(fp);
	fclose(new_fp);

	if(remove(file_path)){
		printf("file not removed succesfully\n");
	}

	if(rename("tmp.txt",file_path)){
		printf("error file not renamed\n");
	}

	return 0;
}
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[]){
	 if(nrhs!=3) 
      		mexErrMsgIdAndTxt( "MATLAB:revord:invalidNumInputs",
            "One input required.");
     if(find_replace_string(mxArrayToString(prhs[0]),mxArrayToString(prhs[1]),mxArrayToString(prhs[2]))){
     	plhs[0] = mxCreateString("error");
     }
     else{
     	plhs[0] = mxCreateString("success");
     }
     
}


int main(int argc ,char* argv[]){

	if(argc<3){
		printf("Invalid input arguments\n");
		printf("Usage: ./replace_string.exe \"string to change\" \"new string\" \"file path\" ");
		return -1;
	}
	find_replace_string(argv[1],argv[2],argv[3]);

	return 0;
}
 





