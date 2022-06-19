#pragma once
#ifndef _CONSTNUMS_H_
#define _CONSTNUMS_H_

#define   IMR_HEADER             512                // imr header is a total of 512 bytes long
#define   IMR_RAW_DATA           32                 // imr raw data is a total of 32 bytes long epoch


#define   OMEGA      7.2921151467e-5                               // rotational angular velocity of the earth
#define   PI         3.1415926535897932384626433832795
#define   C_SPEED    2.99792458e8
#define   D2R        PI / 180.0
#define   R2D        180.0 / PI

/* Physical parameters of the Earth, Sun and Moon  */
#define   R_WGS84    6378137.0          /* Radius Earth [m]; WGS-84  */
#define   B_WGS84    6356752.3141       /* Radius(B) Earth [m]; WGS-84  */
#define   F_WGS84    1.0/298.257223563  /* Flattening; WGS-84   */
#define   Omega_WGS  7.2921151467e-5    /*[rad/s], the earth rotation rate */
#define   GM_Earth   398600.5e+9        /* [m^3/s^2]; WGS-84 */
#define   GM_JGM3    398600.4418e+9     /* [m^3/s^2]; JGM3  */
#define   E2_WGS84   0.0066943799901413
#define   GammaA     9.7803253359
#define   GammaB     9.8321849379


/* Physical parameters of the Earth, Sun and Moon  */
#define   R_CGS2K    6378137.0          /* Radius Earth [m]; CGCS2000  */
#define   F_CGS2K    1.0/298.257222101  /* Flattening; CGCS2000   */
#define   E2_CGS2K   0.0066943800229008 
#define   GM_BDS     398600.4418e+9     /* [m^3/s^2]; CGCS2000  */


#endif