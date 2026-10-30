#include "functions/Backend_Agent.h"
#include <cmath>

// Transform receiver coordinates to VN2000; RTCM payloads are not inspected.
namespace {
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_B = 6356752.31424518;
constexpr double WGS84_E2 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);
constexpr double ARCSEC_TO_RAD = 3.14159265358979323846 / (180.0 * 3600.0);

struct ProvinceProjection {
    const char *code;
    const char *name;
    double centralMeridianDeg;
    double scale;
};

constexpr ProvinceProjection VN2000_PROVINCES[] = {
    {"AG_KG", "An Giang + Kien Giang", 104.75, 0.9999},
    {"BN_BG", "Bac Ninh + Bac Giang", 107.0, 0.9999},
    {"CM_BL", "Ca Mau + Bac Lieu", 104.5, 0.9999},
    {"CAO_BANG", "Cao Bang", 105.75, 0.9999},
    {"DAKLAK_PY", "Dak Lak + Phu Yen", 108.5, 0.9999},
    {"DIEN_BIEN", "Dien Bien", 103.0, 0.9999},
    {"DNAI_BPHUOC", "Dong Nai + Binh Phuoc", 107.75, 0.9999},
    {"DTHAP_TGIANG", "Dong Thap + Tien Giang", 105.0, 0.9999},
    {"GLAI_BDINH", "Gia Lai + Binh Dinh", 108.25, 0.9999},
    {"HA_TINH", "Ha Tinh", 105.5, 0.9999},
    {"HYEN_TBINH", "Hung Yen + Thai Binh", 105.5, 0.9999},
    {"KH_HOA_NTHUAN", "Khanh Hoa + Ninh Thuan", 108.25, 0.9999},
    {"LAI_CHAU", "Lai Chau", 104.75, 0.9999},
    {"LANG_SON", "Lang Son", 107.25, 0.9999},
    {"LCAI_YBAI", "Lao Cai + Yen Bai", 104.75, 0.9999},
    {"LDONG_DNONG_BTHUAN", "Lam Dong + Dak Nong + Binh Thuan", 107.75, 0.9999},
    {"NGHE_AN", "Nghe An", 104.75, 0.9999},
    {"NBINH_HNAM_NDINH", "Ninh Binh + Ha Nam + Nam Dinh", 105.0, 0.9999},
    {"PHU_THO_VPHUC_HBINH", "Phu Tho + Vinh Phuc + Hoa Binh", 104.75, 0.9999},
    {"QNGAI_KTUM", "Quang Ngai + Kon Tum", 108.0, 0.9999},
    {"QNINH", "Quang Ninh", 107.75, 0.9999},
    {"QTRI_QBINH", "Quang Tri + Quang Binh", 106.0, 0.9999},
    {"SON_LA", "Son La", 104.0, 0.9999},
    {"TNINH_LONGAN", "Tay Ninh + Long An", 105.75, 0.9999},
    {"TNGUYEN_BKAN", "Thai Nguyen + Bac Kan", 106.5, 0.9999},
    {"THANH_HOA", "Thanh Hoa", 105.0, 0.9999},
    {"CTHO_STRANG_HGIANG", "Can Tho + Soc Trang + Hau Giang", 105.0, 0.9999},
    {"DANANG_QNAM", "Da Nang + Quang Nam", 107.75, 0.9999},
    {"HA_NOI", "Ha Noi", 105.0, 0.9999},
    {"HPHONG_HDUONG", "Hai Phong + Hai Duong", 105.75, 0.9999},
    {"HCM_BRVT_BDUONG", "Ho Chi Minh + Ba Ria Vung Tau + Binh Duong", 105.75, 0.9999},
    {"HUE", "Hue", 107.0, 0.9999},
    {"TQUANG_HGIANG", "Tuyen Quang + Ha Giang", 106.0, 0.9999},
    {"VLONG_BTRE_TVINH", "Vinh Long + Ben Tre + Tra Vinh", 105.5, 0.9999},
};

struct EcefCoordinate {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

EcefCoordinate llhToEcef(double latitudeDeg, double longitudeDeg, double height) {
    const double latitude = latitudeDeg * 3.14159265358979323846 / 180.0;
    const double longitude = longitudeDeg * 3.14159265358979323846 / 180.0;
    const double sinLatitude = sin(latitude);
    const double cosLatitude = cos(latitude);
    const double radius = WGS84_A / sqrt(1.0 - WGS84_E2 * sinLatitude * sinLatitude);
    return {
        (radius + height) * cosLatitude * cos(longitude),
        (radius + height) * cosLatitude * sin(longitude),
        (radius * (1.0 - WGS84_E2) + height) * sinLatitude,
    };
}

void ecefToLlh(const EcefCoordinate &coordinate, double &latitudeRad,
               double &longitudeRad, double &height) {
    const double ep2 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B)
        / (WGS84_B * WGS84_B);
    const double p = sqrt(coordinate.x * coordinate.x + coordinate.y * coordinate.y);
    if (p < 1e-12) {
        latitudeRad = copysign(3.14159265358979323846 / 2.0, coordinate.z);
        longitudeRad = 0.0;
        height = fabs(coordinate.z) - WGS84_B;
        return;
    }
    const double theta = atan2(coordinate.z * WGS84_A, p * WGS84_B);
    const double sinTheta = sin(theta);
    const double cosTheta = cos(theta);
    latitudeRad = atan2(
        coordinate.z + ep2 * WGS84_B * sinTheta * sinTheta * sinTheta,
        p - WGS84_E2 * WGS84_A * cosTheta * cosTheta * cosTheta);
    longitudeRad = atan2(coordinate.y, coordinate.x);
    const double sinLatitude = sin(latitudeRad);
    const double radius = WGS84_A / sqrt(1.0 - WGS84_E2 * sinLatitude * sinLatitude);
    height = p / cos(latitudeRad) - radius;
}

void blhToUtm(double centralMeridianRad, double scale, double latitudeRad,
              double longitudeRad, double height, double &northing,
              double &easting) {
    const double ep2 = WGS84_E2 / (1.0 - WGS84_E2);
    const double sinLatitude = sin(latitudeRad);
    const double cosLatitude = cos(latitudeRad);
    const double tangent = tan(latitudeRad);
    const double radius = WGS84_A / sqrt(1.0 - WGS84_E2 * sinLatitude * sinLatitude);
    const double t = tangent * tangent;
    const double c = ep2 * cosLatitude * cosLatitude;
    const double a = (longitudeRad - centralMeridianRad) * cosLatitude;
    const double e4 = WGS84_E2 * WGS84_E2;
    const double e6 = e4 * WGS84_E2;
    const double meridian = WGS84_A * (
        (1.0 - WGS84_E2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * latitudeRad
        - (3.0 * WGS84_E2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * sin(2.0 * latitudeRad)
        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * sin(4.0 * latitudeRad)
        - (35.0 * e6 / 3072.0) * sin(6.0 * latitudeRad));
    northing = scale * (meridian + radius * tangent * (
        a * a / 2.0
        + (5.0 - t + 9.0 * c + 4.0 * c * c) * pow(a, 4) / 24.0
        + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * pow(a, 6) / 720.0));
    easting = 500000.0 + scale * radius * (
        a + (1.0 - t + c) * pow(a, 3) / 6.0
        + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * pow(a, 5) / 120.0);
    (void)height;
}

void utmToBlh(double centralMeridianRad, double scale, double northing,
              double easting, double height, double &latitudeRad,
              double &longitudeRad) {
    const double ep2 = WGS84_E2 / (1.0 - WGS84_E2);
    const double x = northing / scale;
    const double y = (easting - 500000.0) / scale;
    const double e4 = WGS84_E2 * WGS84_E2;
    const double e6 = e4 * WGS84_E2;
    const double mu = x / (WGS84_A * (1.0 - WGS84_E2 / 4.0
        - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0));
    const double e1 = (1.0 - sqrt(1.0 - WGS84_E2))
        / (1.0 + sqrt(1.0 - WGS84_E2));
    const double j1 = 3.0 * e1 / 2.0 - 27.0 * pow(e1, 3) / 32.0;
    const double j2 = 21.0 * pow(e1, 2) / 16.0 - 55.0 * pow(e1, 4) / 32.0;
    const double j3 = 151.0 * pow(e1, 3) / 96.0;
    const double j4 = 1097.0 * pow(e1, 4) / 512.0;
    const double footpoint = mu + j1 * sin(2.0 * mu) + j2 * sin(4.0 * mu)
        + j3 * sin(6.0 * mu) + j4 * sin(8.0 * mu);
    const double sinFootpoint = sin(footpoint);
    const double cosFootpoint = cos(footpoint);
    const double tangent = tan(footpoint);
    const double c1 = ep2 * cosFootpoint * cosFootpoint;
    const double t1 = tangent * tangent;
    const double radius = WGS84_A / sqrt(1.0 - WGS84_E2 * sinFootpoint * sinFootpoint);
    const double meridianRadius = radius * (1.0 - WGS84_E2)
        / (1.0 - WGS84_E2 * sinFootpoint * sinFootpoint);
    const double d = y / radius;
    latitudeRad = footpoint - (radius * tangent / meridianRadius) * (
        d * d / 2.0
        - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1 * c1 - 9.0 * ep2) * pow(d, 4) / 24.0
        + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1 * t1
           - 252.0 * ep2 - 3.0 * c1 * c1) * pow(d, 6) / 720.0);
    longitudeRad = centralMeridianRad + (
        d - (1.0 + 2.0 * t1 + c1) * pow(d, 3) / 6.0
        + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1 * c1
           + 8.0 * ep2 + 24.0 * t1 * t1) * pow(d, 5) / 120.0) / cosFootpoint;
    (void)height;
}

bool transformItrfToVn2000(double latitudeDeg, double longitudeDeg, double height,
                           double centralMeridianDeg, double scale,
                           double &localLatitude, double &localLongitude,
                           double &localAltitude, double &northing,
                           double &easting) {
    const EcefCoordinate source = llhToEcef(latitudeDeg, longitudeDeg, height);
    const double omega = 0.00928836 * ARCSEC_TO_RAD;
    const double phi = -0.01975479 * ARCSEC_TO_RAD;
    const double epsilon = 0.00427372 * ARCSEC_TO_RAD;
    const double k = 0.999999747093722;
    const EcefCoordinate transformed = {
        191.90441429 + k * (source.x + epsilon * source.y - phi * source.z),
        39.30318279 + k * (-epsilon * source.x + source.y + omega * source.z),
        111.45032835 + k * (phi * source.x - omega * source.y + source.z),
    };

    double latitudeRad = 0.0;
    double longitudeRad = 0.0;
    double transformedHeight = 0.0;
    ecefToLlh(transformed, latitudeRad, longitudeRad, transformedHeight);
    const double centralMeridianRad = centralMeridianDeg * 3.14159265358979323846 / 180.0;
    blhToUtm(centralMeridianRad, scale, latitudeRad, longitudeRad,
             transformedHeight, northing, easting);
    utmToBlh(centralMeridianRad, scale, northing, easting, transformedHeight,
             latitudeRad, longitudeRad);

    const EcefCoordinate vn2000 = llhToEcef(
        latitudeRad * 180.0 / 3.14159265358979323846,
        longitudeRad * 180.0 / 3.14159265358979323846,
        transformedHeight);
    const double reverseOmega = -0.00928836 * ARCSEC_TO_RAD;
    const double reversePhi = 0.01975479 * ARCSEC_TO_RAD;
    const double reverseEpsilon = -0.00427372 * ARCSEC_TO_RAD;
    const double reverseK = 1.000000252906278;
    const EcefCoordinate local = {
        -191.90441429 + reverseK * (vn2000.x + reverseEpsilon * vn2000.y - reversePhi * vn2000.z),
        -39.30318279 + reverseK * (-reverseEpsilon * vn2000.x + vn2000.y + reverseOmega * vn2000.z),
        -111.45032835 + reverseK * (reversePhi * vn2000.x - reverseOmega * vn2000.y + vn2000.z),
    };
    ecefToLlh(local, latitudeRad, longitudeRad, localAltitude);
    localLatitude = latitudeRad * 180.0 / 3.14159265358979323846;
    localLongitude = longitudeRad * 180.0 / 3.14159265358979323846;
    return std::isfinite(localLatitude) && std::isfinite(localLongitude) && std::isfinite(localAltitude)
        && -90.0 <= localLatitude && localLatitude <= 90.0
        && -180.0 <= localLongitude && localLongitude <= 180.0;
}

} // namespace

bool BackendAgent::getProvinceProjection(const String &code, double &centralMeridianDeg,
                                         double &scale, String &name) const {
    for (const ProvinceProjection &province : VN2000_PROVINCES) {
        if (code.equalsIgnoreCase(province.code)) {
            centralMeridianDeg = province.centralMeridianDeg;
            scale = province.scale;
            name = province.name;
            return true;
        }
    }
    return false;
}

bool BackendAgent::transformToVn2000(const double latitude, const double longitude,
                                     const double altitude, const double centralMeridianDeg,
                                     const double scale, double &localLatitude,
                                     double &localLongitude, double &localAltitude,
                                     double &northing, double &easting) {
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || !std::isfinite(altitude)
        || latitude < -90.0 || latitude > 90.0
        || longitude < -180.0 || longitude > 180.0
        || centralMeridianDeg < 100.0 || centralMeridianDeg > 112.0
        || scale < 0.9990 || scale > 1.0010) {
        return false;
    }
    return transformItrfToVn2000(latitude, longitude, altitude, centralMeridianDeg,
                                 scale, localLatitude, localLongitude, localAltitude,
                                 northing, easting);
}
