/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <pybind11/pybind11.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

namespace py = pybind11;

// Headers for binding functions
/**************************************/
// The following comment block is used for
// gr_modtool to insert function prototypes
// Please do not delete
/**************************************/
// BINDING_FUNCTION_PROTOTYPES(
    void bind_bbscrambler_bb(py::module& m);
    void bind_atsc3_config(py::module& m);
    void bind_bch_bb(py::module& m);
    void bind_ldpc_bb(py::module& m);
    void bind_interleaver_bb(py::module& m);
    void bind_modulator_bc(py::module& m);
    void bind_framemapper_cc(py::module& m);
    void bind_pilotgenerator_cc(py::module& m);
    void bind_freqinterleaver_cc(py::module& m);
    void bind_bootstrap_cc(py::module& m);
    void bind_alpbbheader_bb(py::module& m);
    void bind_paprtr_cc(py::module& m);
    void bind_ldmcombiner_cc(py::module& m);
    void bind_ldmframemapper_cc(py::module& m);
    void bind_subframemapper_cc(py::module& m);
    void bind_subbootstrap_cc(py::module& m);
    void bind_cyclicprefixer_cc(py::module& m);
// ) END BINDING_FUNCTION_PROTOTYPES


// We need this hack because import_array() returns NULL
// for newer Python versions.
// This function is also necessary because it ensures access to the C API
// and removes a warning.
void* init_numpy()
{
    import_array();
    return NULL;
}

PYBIND11_MODULE(atsc3_python, m)
{
    // Initialize the numpy C API
    // (otherwise we will see segmentation faults)
    init_numpy();

    // Allow access to base block methods
    py::module::import("gnuradio.gr");

    /**************************************/
    // The following comment block is used for
    // gr_modtool to insert binding function calls
    // Please do not delete
    /**************************************/
    // BINDING_FUNCTION_CALLS(
    bind_bbscrambler_bb(m);
    bind_atsc3_config(m);
    bind_bch_bb(m);
    bind_ldpc_bb(m);
    bind_interleaver_bb(m);
    bind_modulator_bc(m);
    bind_framemapper_cc(m);
    bind_pilotgenerator_cc(m);
    bind_freqinterleaver_cc(m);
    bind_bootstrap_cc(m);
    bind_alpbbheader_bb(m);
    bind_paprtr_cc(m);
    bind_ldmcombiner_cc(m);
    bind_ldmframemapper_cc(m);
    bind_subframemapper_cc(m);
    bind_subbootstrap_cc(m);
    bind_cyclicprefixer_cc(m);
    // ) END BINDING_FUNCTION_CALLS
}