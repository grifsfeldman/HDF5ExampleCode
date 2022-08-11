// HDF5PT2.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include<string>
#include "hdf5.h"
#include<limits>
#define ImageWidth 2040 
#define ImageHeight 2040
#define ImageNum   64 
//State determines which small proof of concept we are running
int main()
{
    hid_t file_id, dataspace_id,dataset_id,attribute_id;
    herr_t status;
    int state;
    std::cout << "1 for Write 0 for Read \n";
    std::cin >> state;
    //Writes out simple ints to HDF5 file
    if (state == 1) {


        hsize_t dims[2];
        int attr_data[2];
        float fattr_data[2]{};
        fattr_data[0] = 69.420;
        fattr_data[1] = 1.23;

        dims[0] = 4;

        dims[1] = 6;
        attr_data[0] = 100;

        attr_data[1] = 200;

        file_id = H5Fcreate("file.h5", H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
        dataspace_id = H5Screate_simple(2, dims, NULL);
        dataset_id = H5Dcreate(file_id, "/dset", H5T_STD_I32BE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        attribute_id = H5Acreate2(dataset_id, "Units", H5T_STD_I32BE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT);
        status = H5Awrite(attribute_id, H5T_NATIVE_FLOAT, fattr_data);
        status = H5Aclose(attribute_id);
        status = H5Dclose(dataset_id);

        status = H5Sclose(dataspace_id);
        status = H5Fclose(file_id);
    }
    //Reads the values written in state 1
    else if(state == 0) {
        int dset_data[4][6];
        herr_t status;
        std::cout << "We reading boys\n";
        file_id = H5Fopen("file.h5", H5F_ACC_RDWR, H5P_DEFAULT);
        dataset_id = H5Dopen(file_id, "/dset", H5P_DEFAULT);
        status = H5Dread(dataset_id, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, dset_data);
        int x = 12 + 12;
    }
    //Writes out ints and floats
    else if (state == 2) {
        std::cout << "We Writing single elements\n";
        int attr_data = 20;
        file_id = H5Fcreate("single.h5", H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
        hsize_t dims[1];

        dims[0] = 1;
        dataspace_id = H5Screate_simple(1, dims, NULL);
        dataset_id = H5Dcreate(file_id, "/SIGNATURE", H5T_STD_I32BE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        //dataset_id = H5Dopen2(file_id, "/signature", H5P_DEFAULT);
        status = H5Dwrite(dataset_id, H5T_NATIVE_INT,H5S_ALL,H5S_ALL,H5P_DEFAULT,&attr_data);
        float FTR = 420.69;
        status = H5Dclose(dataset_id);
        dataset_id = H5Dcreate(file_id, "/FLOAT_VALUE", H5T_IEEE_F32BE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        status = H5Dwrite(dataset_id, H5T_NATIVE_FLOAT,H5S_ALL,H5S_ALL,H5P_DEFAULT,&FTR);
        status = H5Dclose(dataset_id);
        status = H5Sclose(dataspace_id);
        status = H5Fclose(file_id);

    }
    //Reads in ints and floats from state 2
    else if (state == 3) {
        int Sig = -1;
        std::cout << "We reading single elements\n";
        file_id = H5Fopen("single.h5", H5F_ACC_RDWR, H5P_DEFAULT);
        dataset_id = H5Dopen(file_id, "/SIGNATURE", H5P_DEFAULT);
        status = H5Dread(dataset_id, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &Sig);
        status = H5Dclose(dataset_id);
        std::cout << "The sig is " << Sig << std::endl;
        dataset_id = H5Dopen(file_id, "/DNR", H5P_DEFAULT);
        if (dataset_id == -1) {
            std::cout << "Caught incorrect dataset\n";
        }
        status = H5Dclose(dataset_id);
        status = H5Fclose(file_id);
    }
    //Writes out a 100 by 100 array of shorts
    else if (state == 4) {
        std::cout << "We writing big arrays\n";
        file_id = H5Fcreate("BigArray.h5", H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
        short attr_data[100][100];
        for (int i = 0; i < 100; i++) {
            for (int j = 0; j < 100; j++) {
                attr_data[i][j] = 66;
            }
        }
        hsize_t dims[2];
        dims[0] = 100;
        dims[1] = 100;
        dataspace_id = H5Screate_simple(2, dims, NULL);
        dataset_id = H5Dcreate(file_id, "/IMAGEDATA", H5T_STD_U16BE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        status = H5Dwrite(dataset_id, H5T_NATIVE_SHORT ,H5S_ALL,H5S_ALL,H5P_DEFAULT,attr_data);
        status = H5Dclose(dataset_id);
        status = H5Sclose(dataspace_id);
        dims[0] = 1;
        dataspace_id = H5Screate_simple(1, dims, NULL);
        dataset_id = H5Dcreate(file_id, "/FLOAT_VALUE", H5T_IEEE_F32BE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        status = H5Dclose(dataset_id);
        status = H5Sclose(dataspace_id);
        status = H5Fclose(file_id);
    }
    //Reads in a 100 by 100 array of shorts from state 4
    if (state == 5) {
        std::cout << "We reading an array\n";
        short attr_data[200][100];
        file_id = H5Fopen("BigArray.h5", H5F_ACC_RDWR, H5P_DEFAULT);
        dataset_id = H5Dopen(file_id, "/IMAGEDATA", H5P_DEFAULT);
        status = H5Dread(dataset_id, H5T_NATIVE_SHORT, H5S_ALL, H5S_ALL, H5P_DEFAULT, attr_data);
        status = H5Dclose(dataset_id);
        status = H5Fclose(file_id);

    }
    //Writes out simulated image data. Relies on defines at top to determine the size of the array being written. Data is generic 
    if (state == 6) {
        unsigned short buffer[2040];
        int imageindex = 0;
        std::string DataName = "Image_";
        std::cout << "We pretending to write data\n";
        file_id = H5Fcreate("ImageData.h5", H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
        unsigned short* Image = new unsigned short[ImageHeight*ImageWidth];
        for (int k = 0; k < ImageNum; k++) {

            for (int i = 0; i < ImageWidth * ImageWidth; i++) {
                  Image[i]= 12 + imageindex;

            }
            hsize_t dims[1];
            dims[0] = ImageHeight * ImageWidth;
            dataspace_id = H5Screate_simple(1, dims, NULL);
            std::string DATATYPENAME = DataName + std::to_string(imageindex);
            dataset_id = H5Dcreate(file_id, DATATYPENAME.c_str(), H5T_STD_U16BE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            status = H5Dwrite(dataset_id, H5T_NATIVE_SHORT, H5P_DEFAULT, H5S_ALL, H5P_DEFAULT, Image);
            status = H5Dclose(dataset_id);
            status = H5Sclose(dataspace_id);
            imageindex++;
        }
        delete []Image;

        status = H5Fclose(file_id);

    }
    //Reads in the images using the defined datas at top. Could remap to rely on generic metadata
    if (state == 7) {
        std::string DataTemplate= "Image_";
        unsigned short* DataFrames[ImageNum];
        for (int i = 0; i < ImageNum; i++) {
            DataFrames[i] = new unsigned short[ImageWidth * ImageHeight];
        }
        file_id = H5Fopen("ImageData.h5", H5F_ACC_RDONLY, H5P_DEFAULT);
        for (int i = 0; i < ImageNum; i++) {
            std::string DataName = DataTemplate+ std::to_string(i);
            dataset_id = H5Dopen(file_id, DataName.c_str(), H5P_DEFAULT);
            status = H5Dread(dataset_id, H5T_NATIVE_USHORT, H5S_ALL, H5S_ALL, H5P_DEFAULT, DataFrames[i]);
            status = H5Dclose(dataset_id);
        }

        status = H5Fclose(file_id);

        for (int i = 0; i < ImageNum; i++) {
            delete DataFrames[i];
        }
    }


}
/*
#include "hdf5.h"
#define FILE "file.h5"

int
main()
{

    hid_t  file_id, dataset_id;  //identifiers 
    herr_t status;
    int    i, j, dset_data[4][6];

    //Initialize the dataset. 

    // Open an existing file. 
    file_id = H5Fopen(FILE, H5F_ACC_RDWR, H5P_DEFAULT);

    // Open an existing dataset. 
    dataset_id = H5Dopen2(file_id, "/dset", H5P_DEFAULT);

    // Write the dataset. 
    //status = H5Dwrite(dataset_id, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, dset_data);

    status = H5Dread(dataset_id, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, dset_data);

    // Close the dataset. 
    status = H5Dclose(dataset_id);

    /* Close the file. 
    status = H5Fclose(file_id);
}
*/
