/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2019-2033, Audio Video coding Standard Workgroup of China
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of Audio Video coding Standard Workgroup of China
*    nor the names of its contributors maybe used to endorse or promote products
*    derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "TEncCfg.h"
#include "common/contributors.h"

///< \in TLibEncoder \{

/**
 * Implementation of TEncCfg
 * encoder configuration class
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

Bool TEncCfg::parseCfg(Int argc, TChar* argv[]) {
  Bool isHelp = false;
  Bool ret = 0;

  TComParamParser parser;
  // clang-format off
  parser.addParameter() ///< data, default value, short option, long option, description
    ("Generic")
    (isHelp,                             false,       "h",     "help",                          "help")
    (ConfigParser,                                    "c",     "config",                        "config file")
    (m_inputFileName,                    string(""),  "i",     "input",                         "original PLY input file name")
    (m_bitstreamFileName,                string(""),  "b",     "bitstream",                     "bitstream output file name")
    (m_reconFileName,                    string(""),  "r",     "recon",                         "reconstructed PLY output file name")
    (m_numOfFrames,                      (UInt)1,     "ftbc",  "frames_to_be_coded",            "number of frames to be coded. Default: 1")
    (m_colorTransformFlag,               false,       "ctf",   "color_transform_flag",          "apply color transform method. 1: on, 0: off")
    (m_writePlyInAsciiFlag,              true,        "awf",   "ascii_write_flag",              "recon ply write mode. 1: ascii, 0: binary")
    (m_MD5FileName,                      string(""),  "mdf",   "md5_file_name",                 "name of MD5 file")
    (m_sliceFlag,                        true,        "sf",    "slice_flag",                    "apply slice division. 1: on, 0: off")
    (m_maxPointNumOfSlicesLog2,          (UInt)20,    "spmn",  "slice_point_max_numlg2",        "max number of points in every slice")
    (m_sliceDivisionMode,                (UInt)2,     "sdm",   "slice_division_mode",           "select the mode of slice division")
    ("Geometry")
    (m_hls.sps.geomQuantStep,            1.0f,        "gqs",   "geom_quant_step",               "geometry quantization step")
    (m_hls.sps.geomRemoveDuplicateFlag,  true,        "grdf",  "geom_remove_dup_flag",          "manipulating duplicate points. 1: remove, 0: keep")
    (m_hls.gps.lcuNodeSizeLog2,          (UInt)0,     "lcu",   "geom_lcu_nzlg2",                "the LCU size for node-based geometry coding��0: disabled, >0: enabled" )
    (m_hls.gps.lcuNodeDepth,             (UInt)0,     "lnd",   "geom_lcu_node_depth",           "the depth of a LCU in Octree for node-based geometry. When lcuNodeSizeLog2 > 0. The flag has no effect; otherwise,the actual lcuNodeSizeLog2 = maxNodeSize + 1 - lcuNodeDepth" )
    (m_hls.gps.geomTreeType,             (UInt)1,     "gtt",   "geom_tree_type",                "the tree type for node-based geometry coding��0: octree, 1: tsp" )
    (m_hls.gps.geomTreeSortMode,         (UInt)1,     "gtsm",  "geom_tree_sort_mode",           "sort method before predtree/tsp coding��0: NoSort, 1: MortonSort" )
    (m_hls.gps.log2geomTreeMaxSizeMinus8,(UInt)3,     "lgtmsm8","log2_geom_tree_max_size_minus8","maximum geometry tree size in terms of number of points" )
    (m_hls.gps.geomTreeDensityLow,       1e-9,        "gtdl",  "geom_tree_density_low",         "geomtry tree density low threshold used to determine whehter the octree is used. < geomTreeDensityLow or > geomTreeDensityLow => octree is used; otherwise, predtree is used." )
    (m_hls.gps.geomTreeDensityHigh,      1e-7,        "gtdh",  "geom_tree_density_high",        "geomtry tree density high threshold used to determine whehter the octree is used. < geomTreeDensityLow or > geomTreeDensityLow => octree is used; otherwise, predtree is used." )
    (m_hls.gps.im_qtbt_flag,             true,        "ipf",   "geom_im_qtbt_flag",             "control flag for geometry implicit QTBT partition")
    (m_hls.gps.im_qtbt_num_before_ot,    (UInt)0,     "ipk",   "geom_im_qtbt_k",                "max num of OT before implicit QTBT")
    (m_hls.gps.im_qtbt_min_size,         (UInt)0,     "ipm",   "geom_im_qtbt_m",                "min size of implicit QTBT")
    (m_hls.gps.OccupancymapSizelog2,     (UInt)8,     "OSl",   "geom_ocmap_search_range_edge_log2", "control size of occupancymap")
    (m_hls.gps.singleModeFlag,           true,        "smf",   "geom_single_mode_flag",         "control flag for single point encode mode")
    (m_hls.gps.saveStateFlag,            false,       "stf",   "save_state_flag",               "control whether to save coding state for parallel encoding")
    (m_hls.gps.planarSeqEligible,        false,       "pse",   "geom_planar_eligible",          "0:  planar mode is not eligible; 1: planar mode is eligible")
    ("Attribute")
    (m_hls.sps.recolorMode,              true,        "rm",    "recolor_mode",                  "select recolor mode: 0: regular recolor 1: fast recolor")
    (m_hls.sps.attrPresentFlag,          true,        "apf",   "attr_present_flag",             "0: disable attribute coding, 1: enable attribute coding ")
    (m_hls.sps.colorQuantParam,          (UInt)1,     "cqp",   "color_quant_param",             "color quantization Parameter")
    (m_hls.sps.reflQuantParam,           (UInt)1,     "rqp",   "refl_quant_param",              "reflectance quantization Parameter")
    (m_hls.sps.maxNumAttrMinus1,         (UInt)1,     "mnam1", "max_num_attr_minus1",           "0~15: 'mnam1 + 1' means maximum numbers of attribute categories")
    (m_hls.aps.crossComponentPred,       false,       "ccp",   "cross_comp_pred",               "predict residuals of other components from the residual of the first component")
    (m_hls.aps.orderSwitch,              true,        "os",    "order_switch",                  "change the coding order between three color components")
    (m_hls.aps.colorOutputDepth,         (UInt)8,     "cod",   "color_output_depth",            "color output attribute bit depth")
    (m_hls.aps.reflOutputDepth,          (UInt)16,    "rod",   "refl_output_depth",             "reflectance output attribute bit depth")
    (m_hls.aps.colorInitGolombOffset,    (UInt)0,     "cigo",  "color_init_golomb_offset",      "offset for color output bit depth in golomb number initilization.")
    (m_hls.aps.reflInitGolombOffset,     (UInt)0,     "rigo",  "refl_init_golomb_offset",       "offset for reflectance output attribute bit depth in golomb number initilization.")
    (m_hls.aps.chromaQpOffsetCb,         (Int)0,      "qocb",  "chroma_qp_offset_cb",           "attribute chroma quantisation offset for cb (relative to luma)")
    (m_hls.aps.chromaQpOffsetCr,         (Int)0,      "qocr",  "chroma_qp_offset_cr",           "attribute chroma quantisation offset for cr (relative to luma)")
    (m_hls.aps.transform,                (UInt)0,     "trans", "transform",                     "select the transform-based attribute encoder")
    (m_hls.aps.transformSegmentSize,     (UInt)0,     "tss",   "transform_segment_size",        "transform segment size, set 0 to use the whole point cloud")
    (m_hls.aps.attrTransformQpDelta,     (UInt)0,     "atqd",  "attr_transform_qp_delta",       "attribute transform qp delta, relateive to attrQuantParam")
    (m_hls.aps.nearestPredParam1,        (UInt)0,     "npp1",  "nearest_pred_param1",           "predictor nearestpoint threshold param1")
    (m_hls.aps.nearestPredParam2,        (UInt)0,     "npp2",  "nearest_pred_param2",           "predictor nearestpoint threshold param2")
    (m_hls.aps.axisBias,                 (UInt)1,     "ab",    "axis_bias",                     "axis bias coefficient for reflectance")
    (m_hls.aps.QpOffsetDC,               (Int)0,      "qodc",  "Qp_Offset_DC",                  "transform  coefficients quantisation offset for dc" )
    (m_hls.aps.QpOffsetAC,               (Int)0,      "qoac",  "Qp_Offset_AC",                  "transform  coefficients quantisation offset for dc" )
    (m_hls.aps.chromaQpOffsetDC,         (Int)0,      "cqodc", "chroma_Qp_Offset_DC",           "transform  coefficients quantisation offset for chroma dc coefficient" )
    (m_hls.aps.chromaQpOffsetAC,         (Int)0,      "cqoac", "chroma_Qp_Offset_AC",           "transform  coefficients quantisation offset for chroma ac coefficient" )
    (m_hls.aps.MaxTransNum,              (UInt)0,     "mtn",   "max_transform_number",          "max transform  number in k transform")
    (m_hls.aps.log2maxNumofCoeffMinus8,  (UInt)0,     "lmncm8","log2_max_number_coefficients_minus8","indicate max number of coefficients used transform")
    (m_hls.aps.log2coeffLengthControlMinus8,(UInt)4,  "lclcm8","log2_coeff_length_control_minus8","run-length control")
    (m_hls.aps.colorReordermode,         true,        "crom",  "color_reorder_mode",            "select the color reordered mode ")
    (m_hls.aps.refReordermode,           true,        "rrom",  "ref_reorder_mode",              "select the reflectance reorder mode")
    (m_hls.aps.refGroupPredict,          true,        "rgp",   "ref_group_predict",             "decide to use first prediction in each group in PredTransform")
    (m_hls.aps.attrEncodeOrder,          true,        "aeo",   "attr_encode_order",             "select attribute encode order: false: color and reflectance true : reflectance and color")  
    (m_hls.aps.crossAttrTypePred,        true,        "catp",  "cross_attr_type_pred",          "predict one attribute from the other different type of attributes ")  
    (m_hls.aps.crossAttrTypePredParam1,  (UInt)6554,   "catpp1","cross_attr_type_pred_param1",  "the param1 used for cross attribute type prediction")  
    (m_hls.aps.crossAttrTypePredParam2,  (UInt)1258291,"catpp2","cross_attr_type_pred_param2",  "the param2 used for cross attribute type prediction") 
    (m_hls.aps.deadZoneLen,              (UInt)2,     "dzlen", "deadZone_Len",                  "when aps.transform == 1, use this parameter to control deadzone length")
    (m_hls.aps.colorInitPredTransRatio,  (Int)2,      "cptr",  "color_init_Pred_Trans_Ratio",   "when aps.transform == 1, use this parameter to control the ratio of transform nodes and predicted nodes")
    (m_hls.aps.refInitPredTransRatio,    (Int)0,      "rptr",  "ref_init_Pred_Trans_Ratio",     "when aps.transform == 1, use this parameter to control the ratio of transform nodes and predicted nodes")
    (m_hls.aps.transResLayer,            false,       "trl",   "trans_Residue_Layer",           "when aps.transform == 1, if true: add reslayer; if false: don't add reslayer")
    (m_hls.aps.log2golombGroupSize,      (UInt)4,     "ggss",  "log2_golomb_group_size",        "Group Size of sliding windows for adaptive golomb entropy coding")
    (m_hls.aps.chromaDeadzoneFlag,       false,       "cdzf",  "chroma_Deadzone_length_Flag",   "adaptive deadzone length setting of chroma dc transform coefficients")
    (m_hls.aps.colorQPAdjustFlag,        true,        "cqpaf", "color_QP_Adjust_Flag",          "adjust color QP per point when transform=2")    
    (m_hls.aps.kFracBits,                (UInt)13,    "k",     "kFracBits",                     "set the bit accuracy of the fixed point calculation")
    (m_hls.aps.predFixedPointFracBit,    (UInt)0,     "pfpfb", "pred_Fixed_Point_Frac_Bit",     "set the number of fractional bits in fixed point representation of the calculation of predictor")
    (m_hls.aps.log2predDistWeightGroupSize, (UInt)0,  "pdwgs", "log2_pred_dist_weight_group_size","Group Size of dist weight of prediction ")

    ("DMetric")
    (m_metricsEnable,                    true,        "m",     "metrics_enable",                "calculate metrics flag. 1: on, 0: off")
    (m_calLossless,                      false,       "cll",   "cal_lossless",                  "if calculate lossless metrics. 1: on, 0 : off")
    (m_symmetry,                         true,        "sy",    "symmetry",                      "if calculate symmetry metrics")
    (m_calColor,                         false,       "cc",    "cal_color",                     "if calculate color metrics. 1: on, 0 : off")
    (m_calReflectance,                   false,       "cr",    "cal_reflectance",               "if calculate reflectance metrics. 1: on, 0 : off")
    (m_peakValue,                        0.0f,        "pk",    "peakvalue",                     "peak value of Geometry PSNR(default = 0)")
    (m_duplicateMode,                    true,        "dp",    "duplicate_mode",                "process duplicated points. 1: average 0 : no process")
    (m_multiNeighbourMode,               true,        "ne",    "multineighbour_mode",           "process same distance neighbours. 1: average 0 : no process")
    (m_showHausdorff,                    true,        "hau",   "show_hausdorff",                "if show hausdorff and hausdorffPSNR. 1: on, 0: off")
    (m_PeakMemoryFlag,                   false,       "pmf",   "peak_memory_flag",              "output peak memory info. 1: on, 0: off")

  ;
  // clang-format on

  parser.initParameters();                            ///< set the default parameters
  parser.parseParameters(argc, (const TChar**)argv);  ///< parsing
  parser.printInvalidParameters(cout);                ///< print warnings

  if (isHelp || argc == 1) {  ///< print help info
    parser.printHelp(cout);
    return false;
  }

  ret = checkParameter();  ///< check the validity of parameters

  parser.printParameters(cout);  ///< print parameter values

  return ret;
}

//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

Bool TEncCfg::checkParameter() {
  Bool isFailed = false;

  isFailed |=
    checkCond(m_bitstreamFileName.length() > 0, "Error: bitstream file must be specified.");
  isFailed |= checkCond(m_inputFileName.length() > 0, "Error: input ply file must be specified.");
  isFailed |= checkCond(m_hls.sps.geomQuantStep > 0, "Error: invalid geometry quantization step.");
  isFailed |= checkCond(m_hls.sps.colorQuantParam >= 0, "Error: invalid color quantization step.");
  isFailed |=
    checkCond(m_hls.sps.reflQuantParam >= 0, "Error: invalid reflectance quantization step.");

  ///< lcuNodeSizeLog2 == 1 is equivilant to disable it
  if (m_hls.gps.lcuNodeSizeLog2 == 1) {
    m_hls.gps.lcuNodeSizeLog2 = 0;
  }

  ///< apply cross-component-prediction only when color transform is not used.
  if (!m_colorTransformFlag) {
    m_hls.aps.crossComponentPred = true;
    m_hls.aps.transResLayer = true;
  }

  ///< when haar transform based attribute coding is used, fix the attrTransformQpDelta w.r.t attrQuantParam for CTC test
  if (m_hls.aps.transform) {
    m_hls.aps.attrTransformQpDelta = 32;
  }

  return isFailed;
}

///< \}
