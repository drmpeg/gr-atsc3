      template <typename entry_t, size_t rows, size_t cols>
      void ldpc_bf_type_b(entry_t (&table)[rows][cols])
      {
        uint16_t max_lut_arraysize = 0;
        const unsigned int pbits = FRAME_SIZE_SHORT - NBCH_6_15;
        const unsigned int q = q_val;

        for (auto& ldpc_lut_index_entry : ldpc_lut_index) {
          ldpc_lut_index_entry = 1; /* 1 for the size at the start of the array */
        }

        for (unsigned int row = 0; row < rows; row++) {
          for (unsigned int n = 0; n < 360; n++) {
            for (unsigned int col = 1; col <= table[row][0]; col++) {
              unsigned int current_pbit = (table[row][col] + (n * q)) % pbits;
              ldpc_lut_index[current_pbit]++;
              if (ldpc_lut_index[current_pbit] > max_lut_arraysize) {
                max_lut_arraysize = ldpc_lut_index[current_pbit];
              }
            }
          }
        }
        if (max_lut_arraysize & 0x1) { /* Optimize for RISC-V */
          max_lut_arraysize++;
        }

        /* Allocate a 2D Array with pbits * max_lut_arraysize
         * while preserving two-subscript access
         * see
         * https://stackoverflow.com/questions/29375797/copy-2d-array-using-memcpy/29375830#29375830
         */
        ldpc_lut_b.resize(pbits);
        ldpc_lut_b_data.resize(pbits * max_lut_arraysize);
        ldpc_lut_b_data[0] = 1;
        ldpc_lut_b[0] = ldpc_lut_b_data.data();
        for (unsigned int i = 1; i < pbits; i++) {
          ldpc_lut_b[i] = ldpc_lut_b[i - 1] + max_lut_arraysize;
          ldpc_lut_b[i][0] = 1;
        }
        uint16_t im = 0;
        for (unsigned int row = 0; row < rows; row++) {
          for (unsigned int n = 0; n < 360; n++) {
            for (unsigned int col = 1; col <= table[row][0]; col++) {
              unsigned int current_pbit = (table[row][col] + (n * q)) % pbits;
              ldpc_lut_b[current_pbit][ldpc_lut_b[current_pbit][0]] = im;
              ldpc_lut_b[current_pbit][0]++;
            }
            im++;
          }
        }
      }

      template <typename entry_t, size_t rows, size_t cols>
      void ldpc_bf_type_a(entry_t (&table)[rows][cols])
      {
        int im = 0;
        int row;
        uint16_t max_lut_arraysize = 0;
        const unsigned int pbits = FRAME_SIZE_SHORT - NBCH_3_15;

        for (auto& ldpc_lut_index_entry : ldpc_lut_index) {
          ldpc_lut_index_entry = 1; /* 1 for the size at the start of the array */
        }

        for (unsigned int row = 0; row < rows; row++) {
          if (im == NBCH_3_15) {
            break;
          }
          for (unsigned int n = 0; n < 360; n++) {
            for (unsigned int col = 1; col <= table[row][0]; col++) {
              if ((im % 360) == 0) {
                unsigned int current_pbit = table[row][col];
                ldpc_lut_index[current_pbit]++;
                if (ldpc_lut_index[current_pbit] > max_lut_arraysize) {
                  max_lut_arraysize = ldpc_lut_index[current_pbit];
                }
              }
              else {
                if (table[row][col] < m1_val) {
                  unsigned int current_pbit = (table[row][col] + (n * q1_val)) % m1_val;
                  ldpc_lut_index[current_pbit]++;
                  if (ldpc_lut_index[current_pbit] > max_lut_arraysize) {
                    max_lut_arraysize = ldpc_lut_index[current_pbit];
                  }
                }
                else {
                  unsigned int current_pbit = m1_val + (table[row][col] - m1_val + (n * q2_val)) % m2_val;
                  ldpc_lut_index[current_pbit]++;
                  if (ldpc_lut_index[current_pbit] > max_lut_arraysize) {
                    max_lut_arraysize = ldpc_lut_index[current_pbit];
                  }
                }
              }
            }
            im++;
          }
        }
        im = 0;
        if (max_lut_arraysize & 0x1) { /* Optimize for RISC-V */
          max_lut_arraysize++;
        }

        /* Allocate a 2D Array with pbits * max_lut_arraysize
         * while preserving two-subscript access
         * see
         * https://stackoverflow.com/questions/29375797/copy-2d-array-using-memcpy/29375830#29375830
         */
        ldpc_lut_a.resize(pbits);
        ldpc_lut_a_data.resize(pbits * max_lut_arraysize);
        ldpc_lut_a_data[0] = 1;
        ldpc_lut_a[0] = ldpc_lut_a_data.data();
        for (unsigned int i = 1; i < pbits; i++) {
          ldpc_lut_a[i] = ldpc_lut_a[i - 1] + max_lut_arraysize;
          ldpc_lut_a[i][0] = 1;
        }
        ldpc_lut_a_aux.resize(pbits);
        ldpc_lut_a_aux_data.resize(pbits * max_lut_arraysize);
        ldpc_lut_a_aux_data[0] = 1;
        ldpc_lut_a_aux[0] = ldpc_lut_a_aux_data.data();
        for (unsigned int i = 1; i < pbits; i++) {
          ldpc_lut_a_aux[i] = ldpc_lut_a_aux[i - 1] + max_lut_arraysize;
          ldpc_lut_a_aux[i][0] = 1;
        }
        for (row = 0; row < (int)rows; row++) {
          if (im == NBCH_3_15) {
            break;
          }
          for (int n = 0; n < 360; n++) {
            for (int col = 1; col <= table[row][0]; col++) {
              if ((im % 360) == 0) {
                unsigned int current_pbit = table[row][col];
                ldpc_lut_a[current_pbit][ldpc_lut_a[current_pbit][0]] = im;
                ldpc_lut_a[current_pbit][0]++;
              }
              else {
                if (table[row][col] < m1_val) {
                  unsigned int current_pbit = (table[row][col] + (n * q1_val)) % m1_val;
                  ldpc_lut_a[current_pbit][ldpc_lut_a[current_pbit][0]] = im;
                  ldpc_lut_a[current_pbit][0]++;
                }
                else {
                  unsigned int current_pbit = m1_val + (table[row][col] - m1_val + (n * q2_val)) % m2_val;
                  ldpc_lut_a[current_pbit][ldpc_lut_a[current_pbit][0]] = im;
                  ldpc_lut_a[current_pbit][0]++;
                }
              }
            }
            im++;
          }
        }
        for (;row < (int)rows; row++) {
          for (int n = 0; n < 360; n++) {
            for (int col = 1; col <= table[row][0]; col++) {
              if ((im % 360) == 0) {
                unsigned int current_pbit = table[row][col];
                ldpc_lut_a_aux[current_pbit][ldpc_lut_a_aux[current_pbit][0]] = im;
                ldpc_lut_a_aux[current_pbit][0]++;
              }
              else {
                if (table[row][col] < m1_val) {
                  unsigned int current_pbit = (table[row][col] + (n * q1_val)) % m1_val;
                  ldpc_lut_a_aux[current_pbit][ldpc_lut_a_aux[current_pbit][0]] = im;
                  ldpc_lut_a_aux[current_pbit][0]++;
                }
                else {
                  unsigned int current_pbit = m1_val + (table[row][col] - m1_val + (n * q2_val)) % m2_val;
                  ldpc_lut_a_aux[current_pbit][ldpc_lut_a_aux[current_pbit][0]] = im;
                  ldpc_lut_a_aux[current_pbit][0]++;
                }
              }
            }
            im++;
          }
        }
      }
