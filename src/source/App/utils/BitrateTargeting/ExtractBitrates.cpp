/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ExtractBitrates.h"
#include <string>
#include <sstream>
#include <cassert>
#include <limits>
#include <map>

namespace
{
  /// Maintains a sum and a sample-count for calculating averages
  class Tally
  {
    public:

      /// Contructs a new zeroed tally
      Tally( ): m_sum( 0.0 ), m_numSlices( 0 ) { }

      /// Adds to the sum and increments the sample-count
      void add( double in ) { ++m_numSlices; m_sum += in; }

      /// \return The calculated average
      double average( ) const { return m_sum / ( double )m_numSlices; }

    private:
      double m_sum;
      unsigned long m_numSlices;
  };

  /// Ignores all of the the characters up to and including a given character
  /// \param line The line being read from
  /// \param iLine The active input stream
  /// \param character The character to ignore up to
  /// \throw POCParseException if the stream goes bad before character is encountered or just after character is encountered
  void ignoreUpTo( const std::string& line, std::istream& iLine, char character )
  {
    while( iLine.good( ) && character != iLine.get( ) )
      ;
    if( !iLine.good( ) )
    {
      throw POCParseException( line );
    }
  }

  /// Extracts the average bitrates for each of the temporal layers from the given log
  /// \param i The input stream that represents the log
  /// \return A map that countains the average bitrates for each temporal layer.  Each pair contains the QP value in first and the average bitrate in second.
  /// \throw POCParseException if an error occured while parsing a POC line
  std::map< unsigned char, double > extractBitratesForQPs( std::istream& i )
  {
    std::map< unsigned char, Tally > tallyMap;

    while( i.good( ) )
    {
      // Initialize variables for this line
      std::string line;
      std::getline( i, line );
      std::istringstream iLine( line );
      if( !iLine.good( ) )
      {
        continue;
      }

      // Ignore the "POC"
      if( iLine.get( ) != 'P' )
      {
        continue;
      }
