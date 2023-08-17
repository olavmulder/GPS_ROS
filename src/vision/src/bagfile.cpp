/*
manually unpack pointclouds from bagfile
check for every folder amount pointclouds
get the gps data of the amount of pointclouds
save gps data in file

use file as input for distance.
*/
#include <cstring>
#include <fstream>
#include <iostream>

#include <dirent.h>
#include <error.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>



int GetAmountFiles(const char* path){
    int file_count = 0;
    DIR * dirp;
    struct dirent * entry;

    dirp = opendir(path); /* There should be error handling after this */
    while ((entry = readdir(dirp)) != NULL) {
        if (entry->d_type == DT_REG) { /* If the entry is a regular file */
            file_count++;
        }
    }
    closedir(dirp);
    return file_count;
}
void GetFileNames(char const *path, int amount, char **arr, char const* curDir){

    if(strcmp(path,curDir) == 0){
        struct dirent **namelist;
        
        char a[] = ".";
        char b[] = "..";
        
        int n;

        n = scandir(path, &namelist, 0, alphasort);
        int i=0;
        if (n < 0)
            perror("scandir");
        else {
            while(n--){
                if(!((strcmp(namelist[n]->d_name, a) == 0) || (strcmp(namelist[n]->d_name, b) == 0))){
                //printf("%s\n", namelist[n]->d_name);
                //strcat(*(arr+i), path);
                *(arr+i)=  namelist[n]->d_name;
                i++;
                }    
            }
        }
    }else{
        struct dirent **namelist;
        
        char a[] = ".";
        char b[] = "..";
        
        int n;

        n = scandir(path, &namelist, 0, alphasort);
        int i=0;
        if (n < 0)
            perror("scandir");
        else {
            while(n--){
                if(!((strcmp(namelist[n]->d_name, a) == 0) || (strcmp(namelist[n]->d_name, b) == 0))){
                strcat(*(arr+i),path);
                strcat(*(arr+i),namelist[n]->d_name);
                i++;
                }    
            }
        }
    }
}


int main(int argc, char** argv){
    
    int amountFolders = atoi(argv[1]);

    int amountFilesInFolder[amountFolders];

    for(int i=0;i<amountFolders; i++){
        char* name;
        sprintf(name, "%d", amountFolders);
        amountFilesInFolder[i] = GetAmountFiles("aa");
        printf("amount %d\n", amountFilesInFolder[i]);
    }
    return 0;
}