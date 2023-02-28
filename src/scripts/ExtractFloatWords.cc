/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <iostream>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

using namespace std;

std::vector<float> extractIntegerWords(string str)
{
    stringstream ss;
 
    /* Storing the whole string into string stream */
    ss << str;
 
    /* Running loop till the end of the stream */
    string temp;
    float found;
    std::vector<float> vac;
    while (!ss.eof() && vac.size() < 2) {
 
        /* extracting word by word from stream */
        ss >> temp;
 
        /* Checking the given word is integer or not */
        if (stringstream(temp) >> found)
            vac.push_back(found);
        }
    return vac;
}
int main()
{
    std::string a = "as das d asd as 23.5as asd 6.4";
    cout << extractIntegerWords(a)[1];

    return 0;
}
