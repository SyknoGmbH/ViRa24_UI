# ViRa24 Evaluation GUI
# Copyright (C) 2023 Sykno GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.


import numpy as np


# function to estimate the offset of inphase and quadrature data
def estimate_offset(Idata, Qdata):
    iq_mat = np.transpose(np.array([Idata, Qdata]))
    cov_mat = np.cov(Idata, Qdata)
    _, ev = np.linalg.eigh(cov_mat)
    iq_mat_rotated = np.matmul(iq_mat, np.transpose(ev))
    ky = np.mean(iq_mat_rotated[:, 1])
    iq_mat_rotated[:, 1] -= ky
    iq_complex_rotated = iq_mat_rotated[:, 0] + 1j * iq_mat_rotated[:, 1]
    kx = np.zeros(int((len(iq_complex_rotated) - 1) * len(iq_complex_rotated) / 2))
    ind = 0
    for m in range(len(iq_complex_rotated)):
        for n in range(m, len(iq_complex_rotated)):
            if m != n:
                try:
                    kx[ind] = (abs(iq_complex_rotated[m]) ** 2 - abs(iq_complex_rotated[n]) ** 2) / \
                              (2 * (iq_complex_rotated[m] - iq_complex_rotated[n]).real)
                except RuntimeWarning:
                    kx[ind] = 0
                ind += 1
    kx = kx[~np.isnan(kx)]
    if kx.size != 0:
        offset = np.matmul([np.median(kx), ky], np.linalg.inv(ev))
    else:
        offset = [0, 0]
    return offset