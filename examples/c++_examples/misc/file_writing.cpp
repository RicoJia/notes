#include <bits/types/FILE.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/file.h>

auto FILE_NAME="file_writing.tmp";
/**
 * Theories:
    - A stream is a flow of data into / out of a program
        - ofstream (write to file), instream (read in file), and fstream (can do both) 
    - void ofstream::open(const char *filename, ios::openmode mode);
        - ios::in: ifstream
        - ios::out ofstream
        - ios::app: append 
        - ios::trunc : truncate, By default, output stream will overwrite the existing file. 
        - Examples 
            std::fstream ffile;
            ffile.open(filename, std::ios::in | std::ios::out); //Input and output at the same time
 */
void vanilla_file_writing(){
    std::ofstream ofile;
    // By default it's truncating
    // ofile.open(FILE_NAME, std::ios::trunc);
    ofile.open(FILE_NAME, std::ios::app);
    if (!ofile){
        std::cerr << "file couldn't be opened!"<< std::endl;
        exit(1);
    }
    ofile << "lol,lodfel\n river of time" << std::endl;
    ofile.close();

    std::ifstream ifile;
    char data[100];
    ifile.open(FILE_NAME);
    //Bad way: - ifstream::>> By default, it will stop at space/new line, 
    ifile >> data;
    // std::cout<<data<<std::endl;
    ifile.close();

    // Good way: use is_open() to test.
    ifile.open(FILE_NAME);
    std::string data_str;
    if (ifile.is_open()){
        while (std::getline(ifile, data_str)) {
            std::cout<<data_str<<std::endl;
        }
    }
    ifile.close();

}

/*\
- Theories:
    - you need filelock: http://www.tkxiong.com/archives/2165
        - lseek (repositions file offset) and write are not atomic
*/
void process_safe_write(){
    FILE* pfile;
    if ((pfile = fopen(FILE_NAME, "aw")) == NULL){
        fclose(pfile);
        return;
    }

    // defined in <sys/file.h>
    if (flock(fileno(pfile), LOCK_EX | LOCK_NB) != 0)
    {
        fclose(pfile);
        return;
    }

    std::string fileData = "file data, file data";
    fwrite(fileData.c_str(), 1, fileData.length(), pfile);
    // 解锁
    if (flock(fileno(pfile), LOCK_UN) != 0)
    {
        fclose(pfile);
        return;
    }
 
    fclose(pfile);
    return;
}

int main()
{
    vanilla_file_writing();
    process_safe_write();
}
