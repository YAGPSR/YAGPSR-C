#pragma once

class ClSVDataDecode
{
public:
	ClSVDataDecode();
	void decode(t_SpaceVehicle *const);

private:
	void decodeSV                (t_SpaceVehicle *const);
	void decodeSubframe          (t_DecodedData *const, const vint_t *const, const int &);
	void find_gps_preamble_align (vint_t *, const vint_t &);
	bool is_gps_frame_aligned    (const vint_t &, const int &);
	bool check_parity            (vint_t *const, const vint_t &, const int &);
	void calculateEphemeris      (t_SpaceVehicle *const, const int64_t &);

protected:
};
