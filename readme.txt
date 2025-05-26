--------------------------------------------------------------------------------

         MADOCALIB: MADOCA-PPP Test Library

--------------------------------------------------------------------------------

# OVERVIEW

MADOCA-PPP test library is a reference implementation of MADOCA-PPP, and it contains a post-processing PPP utility. Please note it doesn't assure anything in terms of its effectiveness or reliability.

MADOCA-PPP is Multi-GNSS Advanced Orbit and Clock Augmentation - Precise Point Positioning service of QZSS.

The library is derived from RTKLIB (version 2.4.3 b34, Ref. Note1) and functions of PPP-AR and message conversion copyrighted by the third party.


# USAGE

Please refere to the MADOCALIB manual for usage of MADOCALIB.


# LICENSE

The MADOCALIB is distributed under the following BSD 2-clause license (http://opensource.org/licenses/BSD-2-Clause) and additional exclusive clauses.
Users are permitted to develop, produce or sell their own non-commercial or commercial 
products utilizing, linking or including MADOCALIB as long as they comply with the license.

   Copyright (C) 2023-2024 Cabinet Office, Japan, All rights reserved.
   Copyright (C) 2024 Lighthouse Technology & Consulting Co. Ltd., All rights reserved.
   Copyright (C) 2023-2024 Japan Aerospace Exploration Agency. All Rights Reserved.
   Copyright (C) 2023-2024 TOSHIBA ELECTRONIC TECHNOLOGIES CORPORATION. All Rights Reserved.
   Copyright (c) 2007-2020, T. Takasu, All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

- The software package includes some companion executive binaries or shared
  libraries necessary to execute APs on Windows. These licenses succeed to the
  original ones of these software.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#UPDATE HISTORY
* 2023/03/31 1.0b - The first public version as beta version
* 2024/03/11 license updated
* 2024/03/31 1.1b - not open to the public
* 2024/07/01 1.2  - The second public version.
                      Support MADOCA-PPP ionospheric corrections.
* 2024/08/23 1.3  - The third public version.
                      Add application of cssr2ssr (convert cssr message file to 
                      ssr message file).
                      Support ambiguity re-initialization by Compact SSR phase 
                      discontinuity indicator.
                      Apply phase bias correction only when performing 
                      ambiguity resolution.
                      Other performance improvements.
* 2024/11/22 1.4  - The 4th public version.
                      Support for multiple L6D file input for rnx2rtkp.
                      Antenna file path can be specified by command line option of rnx2rtkp.
                      Support upper case hour code (%HU-> h : hour code (A=0,B=1,C=2,...,X=23)).
                      Fix bug for incorrect maxframe calculation for L6D decoding.
                      Apply phase bias correction even when ambiguity resolution 
                      is not performed.
* 2025/05/26 1.5  - The 5th public version.
                      PPP processing is performed normally even when an undefined
                      GNSSs are stored in the reserved GNSS ID of the Compact SSR.
                      (See mdccssr.c)

#NOTES

* Note 1: The RTKLIB is available at GitHub:
  https://github.com/tomojitakasu/RTKLIB.git
