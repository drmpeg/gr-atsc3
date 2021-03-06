/*
 * Copyright 2021 Free Software Foundation, Inc.
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
/* BINDTOOL_HEADER_FILE(ldpc_bb.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(29d63b4e64872083f2f505e0a4c5a83d)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <atsc3/ldpc_bb.h>
// pydoc.h is automatically generated in the build directory
#include <ldpc_bb_pydoc.h>

void bind_ldpc_bb(py::module& m)
{

    using ldpc_bb    = ::gr::atsc3::ldpc_bb;


    py::class_<ldpc_bb, gr::block, gr::basic_block,
        std::shared_ptr<ldpc_bb>>(m, "ldpc_bb", D(ldpc_bb))

        .def(py::init(&ldpc_bb::make),
           py::arg("framesize"),
           py::arg("rate"),
           D(ldpc_bb,make)
        )
        



        ;




}








