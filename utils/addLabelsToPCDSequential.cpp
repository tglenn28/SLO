#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

vector<uint16_t> getLabels(string);

int main(int argc, char** argv) 
{
    string oldFilename(argv[1]);
    string root = "/home/tyler/rob530proj/data/pcds_labeled/";
    string branch = oldFilename.substr(33,63);
    string newFilename = root + branch;
    string labelFilename = "/home/tyler/rob530proj/data/labels/" + branch.substr(0,2) + "/labels/" + branch.substr(12,6) + ".label";
    vector<uint16_t> labels = getLabels(labelFilename);

    // argv[2] is the number of points out of 10 that are dropped 
    int n = atoi(argv[2]);

    // cout << root << "\n";
    // cout << branch << "\n";
    cout << newFilename << "\n";

    ifstream oldFile(oldFilename);
    ofstream newFile(newFilename);

    string line;
    if (oldFile.is_open())
    {
        if (newFile.is_open())
        {
            int i = 0;
            getline(oldFile, line);

            //////////////// Variables to control downsampling ////////////////
            bool greaterThanFive = false;
            if (n > 5)
            {
                greaterThanFive = true;
                n = 10 - n;
            }

            int numLinesInHeader = 11;
            int count = 1;
            int writeCount = 0;
            int m;
            if (n > 0)
                m = 10/n;
            else
            {
                m = 10;
            }
            
            int p = 1;
            int numPoints;
            // int numDropped = 0;
            int fullSetsOfTen;
            int rem;
            bool addOne;
            int droppedPointsInRem;
            int pointsInRem;
            int pointsAfterDownsampling;            
            
            ///////////////////////////////////////////////////////////////////
            while(!oldFile.eof())
            {
                
                if (i > 10)
                {
                    if ((count > 1 && !greaterThanFive) || (count <= 1 && greaterThanFive) || n == 0)
                    {
                        // line.pop_back();
                        string str = to_string(labels[i-11]);
                        // cout << "label: " << labels[i-11] << ", labelStr: " << str << "\n";
                        line = line + " " + to_string(labels[i-11]) + "\n";
                        newFile << line;
                        writeCount++;

                    }
                    else int blah = 2;

                    if (count >= m && p < m*n) //numDropped < n)
                    {
                        count = 1;
                    }
                    else if (p > 10)
                    {
                        count = 1;
                        // numDropped = 0;
                        p  = 1;
                    }
                    else count++;
                    p++;

                }
                else if (i == 2)
                {
                    // line.pop_back();
                    line = line + " label" + "\n";
                    newFile << line;
                }
                else if (i == 3)
                {
                    // line.pop_back();
                    line = line + " 4" + "\n";
                    newFile << line;
                }
                else if (i == 4)
                {
                    // line.pop_back();
                    line = line + " U" + "\n";
                    newFile << line;
                }
                else if (i == 5)
                {
                    // line.pop_back();
                    line = line + " 1" + "\n";
                    newFile << line;
                }
                else if (i == 6)
                {
                    numPoints = atoi(line.erase(0,6).c_str());
                    fullSetsOfTen = numPoints/10;
                    rem = numPoints%10;
                    addOne = rem%m > 1;
                    droppedPointsInRem = rem/m;
                    if (addOne && n > 0) // n > 0 condition is needed to make sure the count isn't off by 1 when not downsampling at all
                        droppedPointsInRem++;
                    if (greaterThanFive)
                        pointsInRem = droppedPointsInRem;
                    else
                        pointsInRem = rem - droppedPointsInRem;
                    
                    if (greaterThanFive)
                        pointsAfterDownsampling = fullSetsOfTen*(n) + pointsInRem;
                    else
                        pointsAfterDownsampling = fullSetsOfTen*(10 - n) + pointsInRem;

                    newFile << "WIDTH " << pointsAfterDownsampling << "\n";

                    cout << "Number of points after downsampling: " << pointsAfterDownsampling << "\n";
                }
                else if (i == 9)
                {
                    newFile << "POINTS " << pointsAfterDownsampling << "\n";
                }
                else
                {
                    newFile << line << "\n";
                }
                    
                i++;
                
                getline(oldFile, line);
            }
            cout << "Number of lines in original file: " << i - numLinesInHeader << "\n";
            cout << "Number of lines in new file: " << writeCount << "\n";
            cout << "Percentage written: " << 100*writeCount/float(i) << "%\n";

            oldFile.close();
            newFile.close();
            cout << "Labels added successfully\n";

        }  
        else cout << "Unable to open neew pcd file to write\n";
    }
    else cout << "Unable to open pcd file\n";
}

vector<uint16_t> getLabels(string filename)
{
    streampos size;
    uint16_t label;
    uint16_t instance_id;
    vector<uint16_t> labels;


    ifstream file (filename);
    if (file.is_open())
    {
        file.seekg(0, ios::end);
        size = file.tellg()/4;
        file.seekg(0, ios::beg);
        // cout << "file size: " << size << "\n";
        labels.resize(size);

        // for (int i = 0; i < 1000; i++)
        int i = 0;
        while(!file.eof())
        {
            file.read((char *) &label, sizeof(label));
            file.read((char *) &instance_id, sizeof(instance_id));
            // cout << "current point: " << file.tellg() << "\t";
            // cout << "instance id: " << instance_id << "\t";
            // cout << "label: " << label<< "\n";
            int newLabel;
            switch (label) 
            {
                case 0:
                    newLabel = 1;
                    break;
                case 1:
                    newLabel = 2;
                    break;  
                case 10:
                    newLabel = 3;
                    break;
                case 11:
                    newLabel = 4;
                    break;
                case 13:
                    newLabel = 5;
                    break;
                case 15:
                    newLabel = 6;
                    break;
                case 16:
                    newLabel = 7;
                    break;
                case 18:
                    newLabel = 8;
                    break;
                case 20:
                    newLabel = 9;
                    break;
                case 30:
                    newLabel = 10;
                    break;  
                case 31:
                    newLabel = 11;
                    break;
                case 32:
                    newLabel = 12;
                    break;
                case 40:
                    newLabel = 13;
                    break;
                case 44:
                    newLabel = 14;
                    break;
                case 48:
                    newLabel = 15;
                    break;
                case 49:
                    newLabel = 16;
                    break;
                case 50:
                    newLabel = 17;
                    break;
                case 51:
                    newLabel = 18;
                    break;  
                case 52:
                    newLabel = 19;
                    break;
                case 60:
                    newLabel = 20;
                    break;
                case 70:
                    newLabel = 21;
                    break;
                case 71:
                    newLabel = 22;
                    break;
                case 72:
                    newLabel = 23;
                    break;
                case 80:
                    newLabel = 24;
                    break;
                case 81:
                    newLabel = 25;
                    break;
                case 99:
                    newLabel = 26;
                    break;  
                case 252:
                    newLabel = 27;
                    break;
                case 253:
                    newLabel = 28;
                    break;
                case 254:
                    newLabel = 29;
                    break;
                case 255:
                    newLabel = 30;
                    break;
                case 256:
                    newLabel = 31;
                    break;
                case 257:
                    newLabel = 32;
                    break;
                case 258:
                    newLabel = 33;
                    break;
                case 259:
                    newLabel = 34;
                    break;
                default:
                    cout << "Error: an unrecognized class label was encountered\n";
            }

            // cout << "label: " << label << " newLabel: " << newLabel << "\n";
            if (newLabel == 0)
            {
                cout << "Error: new label has a value of 0\n";
            }

            labels[i] = newLabel;
            i++;
        }
        
        file.close();
       
    }
    else cout << "Unable to open label file\n";
    return labels; 
}