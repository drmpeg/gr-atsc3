/*
 * Copyright 2023 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(framemapper_cc.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(edd4218b2f69be7c60f1aadd46e0e1a1)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <atsc3/framemapper_cc.h>
// pydoc.h is automatically generated in the build directory
#include <framemapper_cc_pydoc.h>

void bind_framemapper_cc(py::module& m)
{

    using framemapper_cc    = ::gr::atsc3::framemapper_cc;


    py::class_<framemapper_cc, gr::block, gr::basic_block,
        std::shared_ptr<framemapper_cc>>(m, "framemapper_cc", D(framemapper_cc))

        .def(py::init(&framemapper_cc::make),
           py::arg("framesize"),
           py::arg("rate"),
           py::arg("fecmode"),
           py::arg("constellation"),
           py::arg("fftsize"),
           py::arg("numpayloadsyms"),
           py::arg("numpreamblesyms"),
           py::arg("guardinterval"),
           py::arg("pilotpattern"),
           py::arg("pilotboost"),
           py::arg("firstsbs"),
           py::arg("fimode"),
           py::arg("timode"),
           py::arg("tidepth"),
           py::arg("tiblocks"),
           py::arg("tifecblocksmax"),
           py::arg("tifecblocks"),
           py::arg("cred"),
           py::arg("flmode"),
           py::arg("flen"),
           py::arg("tifmode"),
           py::arg("misomode"),
           py::arg("paprmode"),
           py::arg("l1bmode"),
           py::arg("l1dmode"),
           D(framemapper_cc,make)
        )
        



        ;




}








