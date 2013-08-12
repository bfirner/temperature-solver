/*
 * Copyright (c) 2012 Bernhard Firner and Rutgers University
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 * or visit http://www.gnu.org/licenses/gpl-2.0.html
 */

/*******************************************************************************
 * @file binary_state_solver.cpp
 * Offer binary solutions to the world model based upon binary sensors,
 * such as door switches, on/off power switches, etc.
 *
 * @author Bernhard Firner
 ******************************************************************************/

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <time.h>
#include <unistd.h>
#include <utility>
#include <vector>
#include <sstream>

//Handle interrupt signals to exit cleanly.
#include <signal.h>

#include <owl/netbuffer.hpp>
#include <owl/solver_aggregator_connection.hpp>
#include <owl/solver_world_connection.hpp>
#include <owl/sample_data.hpp>
#include <owl/world_model_protocol.hpp>
#include <owl/grail_types.hpp>

#include <owl/client_world_connection.hpp>

using namespace aggregator_solver;

using std::pair;

using world_model::Attribute;
using world_model::grail_time;
using world_model::URI;

//Global variable for the signal handler.
bool interrupted = false;
//Signal handler.
void handler(int signal) {
  psignal( signal, "Received signal ");
  if (interrupted) {
    std::cerr<<"Aborting.\n";
    // This is the second time we've received the interrupt, so just exit.
    exit(-1);
  }
  std::cerr<<"Shutting down...\n";
  interrupted = true;
}

std::u16string toU16(const std::string& str) {
  return std::u16string(str.begin(), str.end());
}

std::string toString(const std::u16string& str) {
  return std::string(str.begin(), str.end());
}

int main(int arg_count, char** arg_vector) {
  if (arg_count == 2 and std::string(arg_vector[1]) == "-?") {
    std::cout<< "name: Temperature Solver\n";
    std::cout<< "arguments: worldmodel wm_solver wm_client\n";
    std::cout<< "description: Monitors status of simple on/off switches.\n";
    std::cout<< "requires: 'temperature'\n";
    return 0;
  }

  if (4 > arg_count or (arg_count % 2) == 1) {
    std::cerr<<"This program needs 3 arguments:\n";
    std::cerr<<"\t"<<arg_vector[0]<<" <world model ip> <solver port> <client port>\n\n";
    return 0;
  }

  //Set up a signal handler to catch interrupt signals so we can close gracefully
  signal(SIGINT, handler);  

  //World model IP and ports
  std::string wm_ip(arg_vector[arg_count - 3]);
  int solver_port = std::stoi(std::string((arg_vector[arg_count - 2])));
  int client_port = std::stoi(std::string((arg_vector[arg_count - 1])));

  //Set up the solver world model connection;
  std::string origin = "temperature_solver";

  //Read in type information from the config file
  //Types for the GRAIL world model will be read from the file
  std::vector<std::pair<std::u16string, bool>> type_pairs;
	type_pairs.push_back(std::make_pair(u"temperature_celcius", false));

  //Remember what names correspond to what solutions and build a
  //query to find all objects of interest.
  //Use the object to solution map to map transmitters to URIs
  std::map<std::u16string, std::u16string> object_to_solution;
	//Map of transmitter URI (with binary data type) to object URIs.
  std::map<URI, URI> tx_to_uri;

  std::cerr<<"Trying to connect to world model as a solver.\n";
  SolverWorldModel swm(wm_ip, solver_port, type_pairs, toU16(origin));
  if (not swm.connected()) {
    std::cerr<<"Could not connect to the world model as a solver - aborting.\n";
    return 0;
  }

	//Remember switch states so that we only update 
  std::map<URI, double> temp_state;

  //Now handle connecting as a client.
  //Search for sensors attributes of any matching IDs
	URI desired_ids = u".*";
  std::vector<URI> attributes{u"sensor.*"};

	//Update IDs one a second
	world_model::grail_time interval = 1000;

	//Set up parameters for a second request of 'binary state' data.
	//Update as data arrives (interval of 0).
	URI temp_ids = u".*";
	std::vector<URI> temp_attributes{u"temperature"};
	world_model::grail_time temp_interval = 0;

	//We will connect to the world model as a client inside of the processing loop below
	//Whenever we are disconnected we will attempt to reconnect.


  std::cerr<<"Trying to connect to world model as a client.\n";
  ClientWorldConnection cwc(wm_ip, client_port);
	//Send out the requests
	StepResponse sr = cwc.streamRequest(desired_ids, attributes, interval);
	StepResponse temp_response = cwc.streamRequest(temp_ids, temp_attributes, temp_interval);

	std::cerr<<"Starting processing loop...\n";
  while (not interrupted) {
		//Less than operator for two world model attributes
		auto attr_comp = [](const Attribute& a, const Attribute& b) {
			return (a.expiration_date != 0 or a.creation_date < b.creation_date); };
		//Stay connected
    while (not cwc.connected() and not interrupted) {
      std::cerr<<"Waiting 4 seconds before attempting to reconnect client->world model connection\n";
      //Sleep for several seconds after an error before trying to reconnect
      sleep(4);
      cwc.reconnect();
			if (cwc.connected()) {
				//Re-send out the requests
				StepResponse sr = cwc.streamRequest(desired_ids, attributes, interval);
				StepResponse temp_response = cwc.streamRequest(temp_ids, temp_attributes, temp_interval);
			}
    }

		try {

			//Now process the on-demand binary data
			while (temp_response.hasNext() and not interrupted) {
				//Get world model updates
				world_model::WorldState ws = temp_response.next();
				//Check each object for new switch states
				for (const std::pair<URI, std::vector<Attribute>>& I : ws) {
					if (tx_to_uri.end() != tx_to_uri.find(I.first) and
							I.second[0].origin != u"temperature_solver") {
						URI uri = tx_to_uri[I.first];
						//Get the temperature
						double temperature = readPrimitive<double>(I.second[0].data, 0);
						if (temp_state.end() == temp_state.find(uri) or temp_state[uri] != temperature) {
							temp_state[uri] = temperature;
							//Use the object to solution map to get the solution name.
							SolverWorldModel::AttrUpdate soln{u"temperature_celcius", world_model::getGRAILTime(), uri, std::vector<uint8_t>()};
							pushBackVal<double>(temperature, soln.data);
							std::vector<SolverWorldModel::AttrUpdate> solns{soln};
							//Send the data to the world model
							bool retry = true;
							while (retry) {
								try {
									retry = false;
									swm.sendData(solns, false);
								}
								catch (std::runtime_error& err) {
									//Retry if this is just a temporary socket error
									if (err.what() == std::string("Error sending data over socket: Resource temporarily unavailable")) {
										std::cerr<<"Experiencing socket slow down with world model connection. Retrying...\n";
										retry = true;
									}
									//Otherwise keep throwing
									else {
										throw err;
									}
								}
							}
							std::cout<<toString(uri)<<" is "<<temperature<<'\n';
						}
					}
				}
			}
			//Check for responses to map sensors to object identifiers
			while (sr.hasNext() and not interrupted) {
				//Get world model updates
				world_model::WorldState ws = sr.next();
				//Check each object for switch sensor ID information
				for (const std::pair<URI, std::vector<Attribute>>& I : ws) {
					if (I.second.empty()) {
						std::cerr<<toString(I.first)<<" is an empty object.\n";
					}
					else {
						Attribute newest = *(std::max_element(I.second.begin(), I.second.end(), attr_comp));
						if (newest.expiration_date != 0) {
							//TODO FIXME This attribute has been expired so stop updating the
							//status of this ID in the world model
						}
						//Otherwise, make sure that we have signed up for this sensor's data
						//from the aggregators

						//Transmitters are stored as one byte of physical layer and 16 bytes of ID
						grail_types::transmitter tx_switch = grail_types::readTransmitter(newest.data);
						//Map this transmitter to the ID of the object it corresponds to in the world model
						std::string str = std::to_string(tx_switch.phy) + "." + std::to_string(tx_switch.id.lower);
						tx_to_uri[std::u16string(str.begin(), str.end())] = I.first;
						std::cerr<<"Adding "<<std::string(I.first.begin(), I.first.end())<<" into object map with transmitter "<<std::string(str)<<"\n";
					}
				}
			}
		}
		catch (std::runtime_error& err) {
			std::cerr<<"Error in client->world model connection: "<<err.what()<<'\n';
		}
  }
}

