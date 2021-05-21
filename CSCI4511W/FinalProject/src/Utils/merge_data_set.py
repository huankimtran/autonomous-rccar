"""
    Given 2 folders full of images, merge 2 folder into one
    the largest index from the last folder will be added to the indices of images in the second folder
    python3 merge_data_set src1_folder src2_folder dest_folder
"""
import os
import sys
import shutil
import pathlib as pl

if __name__ == '__main__':
    src_1 = pl.Path(sys.argv[1])
    src_2 = pl.Path(sys.argv[2])
    dest = pl.Path(sys.argv[3])
    max_index = 0
    for count, filename in enumerate(os.listdir(str(src_1.absolute()))):
        max_index = max(int(filename[:-4].split('_')[0]), max_index)
        print(f'from {str(src_1.joinpath(filename).absolute())} to {str(src_2.joinpath(filename).absolute())}')
        shutil.copyfile(str(src_1.joinpath(filename).absolute()), str(dest.joinpath(filename).absolute()))
    for count, filename in enumerate(os.listdir(str(src_2.absolute()))):
        tmp = filename[:-4].split('_')
        new_file_name = f'{max_index + int(tmp[0])}_{tmp[1]}_{tmp[2]}.jpg'
        shutil.copyfile(str(src_2.joinpath(filename).absolute()), str(dest.joinpath(new_file_name).absolute()))
    