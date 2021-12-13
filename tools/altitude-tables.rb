#!/usr/bin/env ruby

# tools/altitude-tables: generate lookup tables
#
# Copyright (C) 2021 Ralf Horstmann <ralf@ackstorm.de>
#
# https://github.com/ra1fh/openchronos-ng-elf
#
# This file is part of openchronos-ng-elf.
#
# openchronos-ng-elf is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# openchronos-ng-elf is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

class LookupEntry
    attr_accessor :qnh, :p, :alt

    def to_s
        "qnh=#{@qnh.to_s.rjust(4)} p=#{@p.to_s.rjust(4)} alt=#{("%5.1f" % [@alt]).rjust(8)}"
    end
end

class LookupTable
    attr_accessor :debug, :int

    def initialize(pstep, qstep)
        @debug = false

        @scale = 100

        @p_low = 5000
        @p_high = 11000
        @p_step = pstep

        @qnh_low = 900
        @qnh_high = 1100
        @qnh_step = qstep

        @lut = []
        @lut_row = []

        (@qnh_low..@qnh_high).step(@qnh_step) do |q|
            row = []
            (@p_low..@p_high).step(@p_step) do |p|
                d = LookupEntry.new
                d.alt = alt(p, q)
                d.qnh = q
                d.p   = p
                @lut << d
                row << d
            end
            @lut_row << row
        end
    end

    def generate
        puts ""
        puts "static const int16_t p_low = #{@p_low};"
        puts "static const int16_t p_high = #{@p_high};"
        puts "static const int16_t p_step = #{@p_step};"
        puts "static const int16_t qnh_low = #{@qnh_low};"
        puts "static const int16_t qnh_high = #{@qnh_high};"
        puts "static const int16_t qnh_step = #{@qnh_step};"
        puts ""
        puts "static const int16_t altitude[#{@lut_row.size}][#{@lut_row[0].size}] = {"
        @lut_row.each do |r|
            print "    {   /************* #{r.first.qnh.to_s.rjust(4)} *************/ \n"
            row = r.map { |d| d.alt.round.to_s.rjust(5) }
            while (part = row.take(5)).size > 0
                row = row.drop(5)
                print "        "
                print part.join(", ")
                print ",\n"
            end
            print "    }, \n"
        end
        puts "};"
        puts ""
    end

    def size
        @lut.size
    end

    def dimensions
        print "q: "
        (@qnh_low..@qnh_high).step(@qnh_step) do |q|
            print " #{q}"
        end
        puts
        print "p: "
        (@p_low..@p_high).step(@p_step) do |p|
            print " #{p}"
        end
        puts
    end

    def igradient(a, b)
        grad = nil
        if (a.p != b.p)
            grad = (b.alt.round * @scale - a.alt.round * @scale) / (b.p.round - a.p.round)
            puts "    alt/p   %5.1f" % [grad] if @debug
        end
        if (a.qnh != b.qnh)
            grad = (b.alt.round * @scale - a.alt.round * @scale) / (b.qnh.round - a.qnh.round)
            puts "    alt/qnh %5.1f" % [grad] if @debug
        end
        return grad
    end

    def lookup(p, qnh)
        puts "lookup p=#{p} qnh=#{qnh} alt=#{"%5.1f" % [alt(p, qnh)]}" if @debug

        pindex = (p.to_i - @p_low.to_i) / @p_step.to_i
        qindex = (qnh.to_i - @qnh_low.to_i) / @qnh_step.to_i
        pgrad1 = igradient(@lut_row[qindex][pindex+1],@lut_row[qindex][pindex])
        qgrad1 = igradient(@lut_row[qindex+1][pindex],@lut_row[qindex][pindex])

        pbase = @lut_row[qindex][pindex].p.round
        qbase = @lut_row[qindex][pindex].qnh.round
        abase = @lut_row[qindex][pindex].alt.round * @scale

        alt = abase + (p - pbase) * pgrad1 + (qnh - qbase) * qgrad1
        puts "    alt = %5.1f" % [alt] if @debug

        return alt / @scale
    end

    def alt(p, qnh)
        return 145442.2 * (1.0 - (p * 0.1 / qnh)**0.19)
    end

    def walk(&block)
        @lut_row.each do | row |
            row.each do | item |
                yield(item)
            end
        end
    end
end

class Entry
    attr_accessor :qnh, :p, :alt
    def initialize
    end
end

def analysis(lut)
    debug = false
    results = []
    (5000..10999).step(1) do |p|
        (900..1099).step(1) do |q|
            rec = []
            altexact = lut.alt(p,q)
            altlookup = lut.lookup(p,q)
            rec << p
            rec << q
            rec << altexact
            rec << altlookup
            rec << altexact - altlookup
            results << rec
        end
    end

    abserr = 0.0
    minerr = 0.0
    maxerr = 0.0
    absrec = nil
    maxrec = nil
    minrec = nil
    results.each do |r|
        if r.last.abs > abserr
            abserr = r.last.abs
            maxrec = r
        end
        if r.last > maxerr
            maxerr = r.last
            maxrec = r
        end
        if r.last < minerr
            minerr = r.last
            minrec = r
        end
    end

    puts "abs: #{abserr.inspect}"
    puts "max: #{maxrec.inspect}"
    puts "min: #{minrec.inspect}"

    puts "table = #{lut.size}" if debug
    lut.dimensions if debug
end

def main(argv)
    debug = false
    lut = LookupTable.new(100, 20)
    lut.debug = debug

    lut.walk do |el|
         el.alt += 1
    end

    case argv[0]
    when "gen"
        lut.generate
    when "error"
        analysis(lut)
    else
        puts "usage: altitude-tables <command>"
        puts ""
        puts "possible commands:"
        puts "   gen     generate lookup tables"
        puts "   error   calculate lookup table error"
        puts ""
        exit 1
    end
end

main(ARGV)
