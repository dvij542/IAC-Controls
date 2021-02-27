"""  _
    |_|_
   _  | |
 _|_|_|_|_
|_|_|_|_|_|_
  |_|_|_|_|_|
    | | |_|
    |_|_
      |_|

Author: Souham Biswas
Website: https://www.linkedin.com/in/souham/
Use: A bit unclear (Tentatively to form the original image from from the parameters predicted by the single distortion model with the )
"""

from glob import glob

import numpy as np
import cv2

from geoproj_flow import FlowMapper
from coeffs_condense import coeffs_condense

IN_ROOT_DIRPATH = 'scratchspace'
IN_DIRNAME = 'sample'

COEFFS_SUFFIX = '_coeffs*' + '.npy'
SET_NAME = 'farmland'
OUT_DIRNAME = IN_DIRNAME + '-radicalibv2-outs'
IN_DIRPATH = IN_ROOT_DIRPATH + '/' + IN_DIRNAME + '/' + SET_NAME + '/raw_images'
IN_DIRPATH_UNDIST = IN_ROOT_DIRPATH + '/' + IN_DIRNAME + '/' + SET_NAME + '/undist_images'
IN_DIRPATH_PARAMS = IN_ROOT_DIRPATH + '/' + IN_DIRNAME + '/' + SET_NAME + '/geolabel/cameras.txt'
OUT_DIRPATH = IN_ROOT_DIRPATH + '/' + IN_DIRNAME + '/' + SET_NAME + '/raw_images-radicalibv2-outs'
DISTORT = True
MAP_ORIG = IN_ROOT_DIRPATH + '/' + IN_DIRNAME + '/' + SET_NAME + '/basenames.txt'
OUTPUT_PATH = IN_ROOT_DIRPATH + '/outp/' + SET_NAME + '/raw_images' 
OUTPUT_PATH_UNDIST = IN_ROOT_DIRPATH + '/outp/' + SET_NAME + '/undist_images'
OUTPUT_PATH_PARAMS = IN_ROOT_DIRPATH + '/outp/' + SET_NAME + 'compare_params.txt'

if __name__ == '__main__':
    print(IN_DIRPATH)
    print(OUT_DIRPATH)
    im_fpaths = glob(IN_DIRPATH + "/*")
    # im_paths = glob(IN_DIRPATH_UNDIST + "/*")
    im_fpaths.sort()
    # im_paths.sort()
    print(im_fpaths)
    ext = '.' + im_fpaths[0].split('.')[-1]
    h, w, _ = cv2.imread(im_fpaths[0]).shape
    # undist_gt_test = cv2.imread(im_paths[0])
    # undist_gt_test = cv2.resize(undist_gt_test, (w,h))
    # undist_gt_test = undist_gt_test[:,500:3500,:]
    # cv2.imwrite('./'+str(0)+'-undist_gt.JPG', undist_gt_test)
    # params_gt = np.loadtxt(IN_DIRPATH_PARAMS)[0,-5:-2]
    fm = FlowMapper(None)
    fm.process_dir(IN_DIRPATH, coeffs_only=True, distort = False)

    res_fps = glob(OUT_DIRPATH + '/' + '*' + COEFFS_SUFFIX)
    coeffs = np.array([np.load(fp) for fp in res_fps])
    coeffs_condensed = coeffs_condense(coeffs)

    #print(coeffs_condensed.shape)
    print('Coeffs =', coeffs_condensed)
    # print('Ground truth coeffs =', [params_gt[0],params_gt[1],params_gt[2],1-params_gt[0]-params_gt[1]-params_gt[2]])
    # with open(MAP_ORIG) as f:
    #   content = f.readlines()
    # out_file_params = open(OUTPUT_PATH_PARAMS,"a")
    # out_file_params.write('Coeffs =' + str(coeffs_condensed) + '\n' + 'Ground truth coeffs = ' + str(params_gt))
    # you may also want to remove whitespace characters like `\n` at the end of each line 
    # out_file_params.close()
    # file_names = [x.strip() for x in content]
    #file_names = np.loadtxt(MAP_ORIG)
    # print(file_names)

    fm.gen_undistort_mappings(coeffs_condensed, w, h, repair=True, custom_coeffs=True, src_remap=True)
    #if DISTORT :
    #  im_fpaths = glob('scratchspace/distort_gen/*')
    for i in range(len(im_fpaths)):
        fp = im_fpaths[i]
        # fpe = im_paths[i]
        #print(fp)
        #undistorted_xys = fm.undistort_xys_new1([[50, 100], [200, 193]])
        #undistorted_xys_ = fm.undistort_xys_new2([[50, 100], [200, 193]])
        #undistorted_xys__ = fm.undistort_xys([[50, 100], [200, 193]])
        #print(undistorted_xys)
        #print(undistorted_xys_)
        #print(undistorted_xys__)
        file_name = fp.split('/')[-1][:-4]
        h_d, w_d, _ = cv2.imread(im_fpaths[i]).shape
        if h!=h_d and w!=w_d :
          h = h_d
          w = w_d
          fm.gen_undistort_mappings(coeffs_condensed, w, h, repair=True, custom_coeffs=True, src_remap=True)
        im = cv2.imread(fp)
        # ime = cv2.imread(fpe)
        undistorted_im = fm.undistort_im(im)
        #out_fp = fp.replace(IN_DIRNAME, "outp").replace(ext, '-undistorted_remapped' + ext)
        out_fp = OUTPUT_PATH + '/' + file_name + '-undistorted' + ext
        #out_fp_o = fp.replace(IN_DIRNAME, "outp").replace(ext, '-undistorted_expected' + ext)
        # out_fp_o = fpe.replace(IN_DIRNAME, "outp").replace(ext, '-undistorted_expected' + ext)
        dim = (im.shape[1],im.shape[0])
        # resized = cv2.resize(ime, dim, interpolation = cv2.INTER_AREA)
        print(out_fp)
        # print(out_fp_o)
        out_f = OUTPUT_PATH + '/' + file_name + '-distorted' + ext
        print(out_f)
        #cv2.imshow(undistorted_im)
        cv2.imwrite(out_fp, undistorted_im)
        # cv2.imwrite(out_fp_o, resized)
        cv2.imwrite(out_f,im)
    
