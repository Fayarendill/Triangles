#include <iostream>
#include <packing/stripe.h>
#include <packing/utility/visualiser.h>
#include <memory>
#include <chrono>

int main()
{
	auto stripe = std::make_shared<packing::Stripe>(8);
	packing::utility::StripVisualiser visualiser(15);
	size_t time_0, time_1;

	time_0 = std::chrono::system_clock::now().time_since_epoch() /
		std::chrono::milliseconds(1);
	stripe->pack(packing::Triangle{ 4 });
	stripe->pack(packing::Triangle{ 2.4 });
	stripe->pack(packing::Triangle{ 5.3 });
	stripe->pack(packing::Triangle{ 1.7 });
	stripe->pack(packing::Triangle{ 4 });
	stripe->pack(packing::Triangle{ 2.4 });
	stripe->pack(packing::Triangle{ 5.3 });
	stripe->pack(packing::Triangle{ 1.7 });
	stripe->pack(packing::Triangle{ 1.7 });
	stripe->pack(packing::Triangle{ 4 });
	stripe->pack(packing::Triangle{ 2.4 });
	stripe->pack(packing::Triangle{ 2.4 });
	stripe->pack(packing::Triangle{ 5.3 });
	stripe->pack(packing::Triangle{ 1.7 });
	stripe->pack(packing::Triangle{ 1.7 });
	time_1 = std::chrono::system_clock::now().time_since_epoch() /
		std::chrono::milliseconds(1);
	visualiser.display_strip("img/fig_result_1", stripe->width(), 1.1 * stripe->current_height(), stripe->packing(), stripe->fitting_chain());
	std::cout << "CR 1: " << stripe->current_competetive_ratio() << std::endl;

	std::cout << (time_1 - time_0) / 1000.0 << std::endl;

	stripe = std::make_shared<packing::Stripe>(10);
	time_0 = std::chrono::system_clock::now().time_since_epoch() /
		std::chrono::milliseconds(1);
	stripe->pack(packing::Triangle{ 4 });
	stripe->pack(packing::Triangle{ 2.4 });
	stripe->pack(packing::Triangle{ 1.3 });
	stripe->pack(packing::Triangle{ 3.23 });
	stripe->pack(packing::Triangle{ 1.4 });
	stripe->pack(packing::Triangle{ 3.4 });
	stripe->pack(packing::Triangle{ 5.43 });
	stripe->pack(packing::Triangle{ 2.7 });
	stripe->pack(packing::Triangle{ 1.27 });
	time_1 = std::chrono::system_clock::now().time_since_epoch() /
		std::chrono::milliseconds(1);
	visualiser.display_strip("img/fig_result_2", stripe->width(), 1.1 * stripe->current_height(), stripe->packing(), stripe->fitting_chain());
	std::cout << "CR 2: " << stripe->current_competetive_ratio() << std::endl;
	
	std::cout << (time_1 - time_0) / 1000.0 << std::endl;

	stripe = std::make_shared<packing::Stripe>(6);
	time_0 = std::chrono::system_clock::now().time_since_epoch() /
		std::chrono::milliseconds(1);
	stripe->pack(packing::Triangle{ 1.4 });
	stripe->pack(packing::Triangle{ 2.3 });
	stripe->pack(packing::Triangle{ 1.21 });
	stripe->pack(packing::Triangle{ 1.4 });
	stripe->pack(packing::Triangle{ 1.4 });
	stripe->pack(packing::Triangle{ 4.3 });
	stripe->pack(packing::Triangle{ 5.3 });
	stripe->pack(packing::Triangle{ 2.7 });
	stripe->pack(packing::Triangle{ 1.21 });
	time_1 = std::chrono::system_clock::now().time_since_epoch() /
		std::chrono::milliseconds(1);
	visualiser.display_strip("img/fig_result_3", stripe->width(), 1.1 * stripe->current_height(), stripe->packing(), stripe->fitting_chain());
	std::cout << "CR 3: " << stripe->current_competetive_ratio() << std::endl;
	
	std::cout << (time_1 - time_0) / 1000.0 << std::endl;

	stripe = std::make_shared<packing::Stripe>(20);
	time_0 = std::chrono::system_clock::now().time_since_epoch() /
		std::chrono::milliseconds(1);
	stripe->pack(packing::Triangle{ 11.4 });
	stripe->pack(packing::Triangle{ 4.3 });
	stripe->pack(packing::Triangle{ 3.21 });
	stripe->pack(packing::Triangle{ 4.29 });
	stripe->pack(packing::Triangle{ 1.4 });
	stripe->pack(packing::Triangle{ 13.4 });
	stripe->pack(packing::Triangle{ 4.4 });
	stripe->pack(packing::Triangle{ 14.88 });
	stripe->pack(packing::Triangle{ 1.29 });
	time_1 = std::chrono::system_clock::now().time_since_epoch() /
		std::chrono::milliseconds(1);
	visualiser.display_strip("img/fig_result_4_9", stripe->width(), 1.1 * stripe->current_height(), stripe->packing(), stripe->fitting_chain());
	
	std::cout << "CR 4: " << stripe->current_competetive_ratio() << std::endl;
	std::cout << (time_1 - time_0) / 1000.0 << std::endl;

}