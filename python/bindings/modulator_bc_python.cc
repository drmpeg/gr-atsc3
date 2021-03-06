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
/* BINDTOOL_HEADER_FILE(modulator_bc.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(eb89791b38054dd12a8be9a47ca1510a)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <atsc3/modulator_bc.h>
// pydoc.h is automatically generated in the build directory
#include <modulator_bc_pydoc.h>

void bind_modulator_bc(py::module& m)
{

    using modulator_bc    = ::gr::atsc3::modulator_bc;


    py::class_<modulator_bc, gr::block, gr::basic_block,
        std::shared_ptr<modulator_bc>>(m, "modulator_bc", D(modulator_bc))

        .def(py::init(&modulator_bc::make),
           py::arg("framesize"),
           py::arg("rate"),
           py::arg("constellation"),
           D(modulator_bc,make)
        )
        



        ;




}








