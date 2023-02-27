#pragma once

class ClSVTrack
{
public:
	ClSVTrack();

	void track(t_SpaceVehicle *const, const vcomplex_t &, const int &, const size_t &);

private:
	void trackSV(t_SpaceVehicle *const, const int &);

protected:

};