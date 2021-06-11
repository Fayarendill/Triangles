#include <iostream>
#include <packing/stripe.h>
#include <packing/utility/visualiser.h>

int main()
{
	packing::Stripe stripe(14);
	packing::utility::StripVisualiser visualiser(10);

	stripe.pack(packing::Triangle{ 2 });

	visualiser.display_strip("", 14, 15, stripe.packing());
}