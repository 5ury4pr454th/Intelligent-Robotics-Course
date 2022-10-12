from math import sqrt

def line_btw(c1, c2):
    """Computes the line between two points

    argument : c1 (tuple of float), c2 (tuple of float)
    return : m, c (float)
    """
    m = (c2[1] - c1[1])/(c2[0] - c1[0])
    c = c2[1] - m*c2[0]

    return m, c


def dist_btw(c1, c2):
    """Computes the distance between two points

    argument: c1 (tuple of float), c2 (tuple of float)
    return: d (float)
    """
    d = sqrt((c2[1] - c1[1])**2 + (c2[0] - c1[0])**2)
    return d


def point_perp(c, l):
    """Computes the perpendicular distance between a line and a point

    argument: c (tuple of float) representing a point
              l (tuple of float) representing a line ( (m,c) )
    return: d (float)
    """
    return abs(1 - l[0] - l[1]) / sqrt(1 + l[0]**2)


def dist_btw_polygon(c, p):
    """Computes the shortest perpendicular distance to a line
    argument: c (tuple of float) representing a point
              p (tuple of tuple of float) representing a polygon
    return: d (float)
    """
    min_dist = 999

    for line in p:
        calculated_dist = point_perp(c, line)
        if calculated_dist < min_dist:
            min_dist = calculated_dist

    return min_dist


def tangent_to_polygon(p):
    """Computes the tangent of a point to another polygon
    
        argument: p (tuple of (float, float)) polygon
        return: tangents (list of int) 4 possibles tangent
    """
    
    m0 = {}
    m_vol0 = 9999
    dist_max = 0
    for vertice in p:
        d = dist_btw(p[0], vertice)

        if d > dist_max:
            dist_max = d
            vertice_max = vertice
            index_max = p.index(vertice)

    for vertice in p:
        # tangent = 0 <-> m = +infinite
        m0[p.index(vertice)] = dist_btw(p[-1], vertice) / dist_btw(p[0], vertice)
        vol0 = (m0[p.index(vertice)] * dist_btw(p[0], vertice)**2) / 2
        if vol0 < m_vol0:
            m_vol0 = vol0
            n = p.index(vertice)

    # return the two tangents seperated by a \
    tangents = [index_max, n]
    n = (n + 1) % len(p)
    tangents.append(n)

    n = (n + 1) % len(p)
    if n == (index_max + 1) % len(p):
        pass
    else:
        tangents.append(n)

    return tangents


def polygon_intersection(list_polygons):
    """compute the intersection points from multiple polygons
    
    argument: list_polygons (tuple of tuple of (float, float)) mulitiple polygons
    return: intersection ( tuple of float) intersection points of the polygon
    """

    ordered_polygon = []
    for polygon in list_polygons:
        tangents = tangent_to_polygon(polygon)
        ordered_polygon.append(list(polygon[i] for i in tangents))

    return ordered_polygon
