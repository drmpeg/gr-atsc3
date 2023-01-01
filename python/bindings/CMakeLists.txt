# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT atsc3_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT atsc3_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND atsc3_python_files
    bbscrambler_bb_python.cc
    atsc3_config_python.cc
    bch_bb_python.cc
    ldpc_bb_python.cc
    interleaver_bb_python.cc
    modulator_bc_python.cc
    framemapper_cc_python.cc
    pilotgenerator_cc_python.cc
    freqinterleaver_cc_python.cc
    bootstrap_cc_python.cc
    alpbbheader_bb_python.cc
    paprtr_cc_python.cc
    ldmcombiner_cc_python.cc
    ldmframemapper_cc_python.cc
    subframemapper_cc_python.cc
    subbootstrap_cc_python.cc
    cyclicprefixer_cc_python.cc
    tdmframemapper_cc_python.cc
    fdmframemapper_cc_python.cc python_bindings.cc)

GR_PYBIND_MAKE_OOT(atsc3
   ../..
   gr::atsc3
   "${atsc3_python_files}")

install(TARGETS atsc3_python DESTINATION ${GR_PYTHON_DIR}/atsc3 COMPONENT pythonapi)
