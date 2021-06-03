""" Module: gravityChamber.py
    Author: Ben Stucky
    Date: 7/1/19

This module simulates a gravity chamber containing
some circular falling bodies. Each FallingBody in a GravityChamber object
has customizable gravity (angle and magnitude), air resistance, radius,
initial velocity, position, and collision coefficient with respect to the walls
of the chamber ( 0 = inelastic, 1 = elastic ).

The GravityChamber object has customizable height and width,
and also features customizable friction coefficient (0 = no friction,
1 = all friction ) and collision coefficient (again, 0 = inelastic,
1 = elastic).

Game idea 1: Avoid a "spinning blade" which travels through the chamber
on a track, popping any Bodies it touches.

Game idea 2: Choose initial directions of bodies for given velocities,
so that when the chamber runs the bodies come to rest without touching
certain obstacles.

"""

from graphics import *
from math import *
from random import *
from button import *

DEFAULT_GRAVITY_DIRECTION = -pi / 2
DEFAULT_GRAVITY_ACCELERATION = 20
DEFAULT_AIR_RESISTANCE = 0.0008
DEFAULT_WALL_COLLISION_COEFFICIENT = 0.6
DEFAULT_BODY_COLLISION_COEFFICIENT = 1 # This is currently "artificial,"
                                         # and should be kept close to 1.
                                         # Making it much less than 1 will
                                         # cause very unrealistic-looking
                                         # "sticking" and "stalling" effects.
                                         # The point of having it is to
                                         # take some energy out of the system
                                         # since elastic collisions between
                                         # bodies seem to have an explosive
                                         # effect on the total chamber
                                         # energy.
DEFAULT_FRICTION = 0
DEFAULT_HEIGHT = 700
DEFAULT_WIDTH = 1000
DEFAULT_OFFSET = 1.05
DEFAULT_NUM_OF_BODIES = 0
DEFAULT_RADIUS = min( DEFAULT_HEIGHT, DEFAULT_WIDTH ) / 10
DEFAULT_GRAVITATIONAL_CONSTANT = 0

# Later, determine framelength based on the number of bodies?
DEFAULT_FRAMELENGTH = 1/20

DEBUG = False
"""In DEBUG mode, FallingBodies flash different colors in various situations:
RED flash means Body was beyond the boundary of the chamber - position reset.
ORANGE flash means Body collided with another body.
BLUE flash means Body was inside another - position reset."""

class FallingBody:
    """A class representing a circular body in free fall."""

    maxvel =  ( DEFAULT_HEIGHT + DEFAULT_WIDTH ) # Cap the velocity.
    
    correctionframes = 1 # This isn't doing anything right now.
                         # Later, I want to use it to "smooth out"
                         # position resets.

    def __init__( self, xpos, ypos, xvel, yvel, rad,
                  col, gravangle, gravaccel, res, chamber):
        """Initialize the FallingBody."""
        
        self.rad = rad
        self.xvel = 0
        self.yvel = 0
        self.setVel( xvel, yvel )
        self.col = col # Collision coefficient (0 = inelastic, 1 = elastic)
                       # Again, this used somewhat artificially.

        self.xaccel = cos( gravangle ) * gravaccel
        self.yaccel = sin( gravangle ) * gravaccel
        
        #self.gravangle = gravangle # gravity pulls downward by default
        #self.gravaccel = gravaccel # acceleration due to gravity
        self.res = res # drag coefficient (0 - inf, 0 = vacuum )
        
        
        self.chamber = chamber # GravityChamber containing this body
        self.circle = Circle( Point( xpos, ypos ), rad ) # Circle object
                                                    # representing this
                                                    # FallingBody

        # Color the body a random shade of green by default.
        self.color = color_rgb( randrange( 0, 100 ),
                                randrange( 100, 256 ),
                                randrange( 0, 100 ) )
        self.circle.setFill( self.color )
        return

    def capvels( self, xvel, yvel ):
        """Returns self.xvel and self.yvel capped by self.maxvel."""
        cappedxvel = 0
        cappedyvel = 0
        if abs( xvel ) > self.maxvel:
            cappedxvel = ( abs( xvel ) / xvel ) * self.maxvel
            
        else:
            cappedxvel = xvel
        if abs( yvel ) > self.maxvel:
            cappedyvel = ( abs( yvel ) / yvel ) * self.maxvel
        else:
            cappedyvel = yvel
        return cappedxvel, cappedyvel 

    def drawBody( self ):
        """Redraws (or draws) the falling body."""
        self.circle.draw( self.chamber.getWin() )
        return

    def undrawBody( self ):
        """Undraw this FallingBody."""
        self.circle.undraw()
        return

    def move( self, dx, dy ):
        """Moves a FallingBody by a given amount."""
        self.circle.move( dx, dy )
        return
        
    def update( self, framelength ):
        """Updates the position of the body, taking into account the
            direction of gravity, air resistance."""    

        # Calculate acceleration caused by air resistance
        # using that this acceleration is proportional to velocity squared,
        # and that the acceleration is always in the direction opposite
        # of the velocity.
        xairres = -self.res * self.xvel * abs( self.xvel )
        yairres = -self.res * self.yvel * abs( self.yvel )

        # Calculate acceleration due to gravity:
        #xgravaccel = cos( self.gravangle ) * self.gravaccel
        #ygravaccel = sin( self.gravangle ) * self.gravaccel

        # Calculate change in velocity:
        xvelchange = ( self.xaccel + xairres ) * framelength # v = a * t
        yvelchange = ( self.yaccel + yairres ) * framelength # v = a * t
        
        # Modify velocity and position according to the above calculations:
        xvel1, yvel1 = self.capvels( self.xvel+ xvelchange,
                                     self.yvel + yvelchange )
        movex = framelength * ( self.xvel+ xvel1 ) / 2.0
        movey = framelength * ( self.yvel+ yvel1 ) / 2.0
        self.xvel, self.yvel = xvel1, yvel1
        
        # Redraw self to reflect new position:
        self.circle.move( movex, movey )
        return


    def handleWallCollisions( self, framelength ):
        """This method handles collisions of self with the walls
    of its containing chamber, updating position and velocity
    accordingly."""
        # Determine which walls we are on
        # and position adjustments to make:
        onwalls = self.onWalls( framelength )

        # Calculate collision tolerance.
        # tolerance = self.getTolerance( framelength )

        # Reset position if you are on a wall.
        self.resetPosOnWallCollision( onwalls )

        if "right" in onwalls or "left" in onwalls:
            # Factor in friction (technically should depend on force):
            self.yvel = ( 1 - self.chamber.getFric() ) * self.yvel
            # Bounce according to collision coefficient of chamber:
            self.xvel = -self.xvel * self.col * self.chamber.getCol()
            self.yvel = self.yvel * self.col * self.chamber.getCol()
                
        if "top" in onwalls or "bottom" in onwalls:
            # Factor in friction (technically should depend on force):
            self.xvel = ( 1 - self.chamber.getFric() ) * self.xvel
            # Bounce according to collision coefficient of chamber:
            self.xvel = self.xvel * self.col * self.chamber.getCol()
            self.yvel = -self.yvel * self.col * self.chamber.getCol()

        return

    def onWalls( self, framelength ):
        """Used to check for when the object is on (or beyond)
        the boundary of the chamber.
        getTolerance() is used to make sure the object does not skip through
        the boundary between frames, and is dependent on velocity.
        This check thus depends on framelength."""
        
        tol = self.getTolerance( framelength )
        onwalls = []
        if self.xvel < 0 and ( ( 
            self.getX() - self.rad ) <= tol ):
            # We are on or beyond the left wall.
            onwalls.append( "left" )
        if self.xvel > 0 and ( -( 
            self.getX() - (
            self.chamber.getWidth() - self.rad ) ) <= tol ):
            # We are on or beyond the right wall.          
            onwalls.append( "right" )
        if self.yvel < 0 and ( ( 
            self.getY() - self.rad  ) <= tol ):         
            # We are on or beyond the bottom wall.          
            onwalls.append( "bottom" )
        if self.yvel > 0 and ( -( 
            self.getY() - ( self.chamber.getHeight() -
            self.rad ) ) <= tol ):
            # We are on or beyond the top wall.
            onwalls.append( "top" )
        return onwalls

    def resetPosOnWallCollision( self, onwalls ):
        """Resets the position of the FallingBody if it has moved
    beyond the boundary of the chamber."""
        # Return immediately if possible.
        if not onwalls:
            return

        # Color the Body red on correction if in DEBUG mode.
        if DEBUG:
            self.setFill( "red" )
        
        xdiff = 0
        ydiff = 0        
        if "left" in onwalls:
            xdiff += self.rad - self.getX()
        if "right" in onwalls:
            xdiff += ( self.chamber.getWidth() - self.rad ) - self.getX()
        if "bottom" in onwalls:
            ydiff += self.rad - self.getY()
        if "top" in onwalls:
            ydiff += ( self.chamber.getHeight() - self.rad ) - self.getY()

        # Smoothly move the FallingBody back in bounds
        # in self.correctionframes steps.
        for i in range( self.correctionframes ):
            self.circle.move( xdiff / self.correctionframes,
                              ydiff / self.correctionframes )
        return
        
    def getY( self ):
        """Returns ypos of the FallingBody"""
        return self.circle.getCenter().getY()

    def getX( self ):
        """Returns xpos of the FallingBody"""
        return self.circle.getCenter().getX()

    def getXvel( self ):
        """Returns horizontal velocity of the FallingBody."""
        return self.xvel

    def getYvel( self ):
        """Returns vertical velocity of the FallingBody."""
        return self.yvel

    def setVel( self, xvelnew, yvelnew ):
        """Set the velocity of the FallingBody."""
        self.xvel, self.yvel = self.capvels( xvelnew, yvelnew ) 
        return

    def setAccel( self, xaccel, yaccel ):
        self.xaccel = xaccel
        self.yaccel = yaccel
        return

    #def setGravAngle( self, angle ):
    #    """Setter for angle of gravity of this FallingBody."""
    #    self.gravangle = angle
    #    return

    #def setGravAccel( self, accel ):
    #   """Setter for acceleration of gravity of this FallingBody."""
    #    self.gravaccel = accel
    #    return

    def getGravAccel( self ):
        """Return the gravity acceleration of this FallingBody."""
        return sqrt( self.xaccel**2 + self.yaccel**2 )

    def setGravity( self, angle, accel ):
        """Set the gravity direction and angle of this FallingBody."""
        self.xaccel = cos( angle ) * accel
        self.yaccel = sin( angle ) * accel
        #self.setGravAngle( angle )
        #self.setGravAccel( accel )
        return

    def getColor( self ):
        """Returns the color of this FallingBody."""
        return self.color

    def setColor( self, color ):
        """Setter for color of this falling body."""
        self.color = color
        self.circle.setFill( self.color )
        return

    def getGravAngle( self ):
        """Getter for angle of gravity of this FallingBody."""
        return acos( self.xaccel / ( self.xaccel**2 + self.yaccel**2 ) ) 

    def setAirRes( self, airres ):
        """Setter for air resistance of this FallingBody."""
        self.res = airres
        return

    def getCol( self ):
        """Returns collision coefficient of the FallingBody."""
        return self.col

    def setBodyCol( self, col ):
        """Setter for collision coefficient of this FallingBody."""
        self.col = col
        return

    def getRad( self ):
        """Returns radius of the FallingBody"""
        return self.rad

    def setFill( self, color ):
        """Set the color of the FallingBody."""
        self.circle.setFill( color )
        return    

    def stopped( self ):
        """Decides if object has come to rest,
    including when an object is at its max height
    and no velocity component tranverse to gravity."""
        return self.getSpeed() == 0

    def getSpeed( self ):
        """Returns the absolute speed of the FallingBody."""
        return sqrt( self.xvel**2 + self.yvel**2 )
    
    def getTolerance( self, framelength ):
        """Tolerance should be proportional to speed:
        This is bounded below by half of the distance traveled
        by the body in one frame."""
        #return max( self.getSpeed() * framelength,
        #            self.gravaccel * framelength ) / 2
        return self.getSpeed() * framelength / 2


class GravityChamber:
    """A class to represent a chamber with gravity."""

    def __init__( self, width, height, offset, col, fric, numbodies,
                  gravangle, gravaccel, gravconst, rad, airres, bodycol ):
        """Initializes the GravityChamber of provided width and height."""
        self.offset = offset # How much bigger is window than chamber?
        self.height = height
        self.width = width
        self.col = col # collision coefficient of walls
                      # (0 = inelastic, 1 = elastic)
        self.fric = 0 #0.0006 # friction coefficient of walls
                      # (0 = no friction, 1 = all friction )
                      # Technically this should depend on the force of the
                      # FallingBody on the boundary

        self.gravangle = gravangle
        self.gravaccel = gravaccel
        self.gravconst = gravconst

        self.zeroGravity = False

        self.bodies = [] # Array containing the FallingBodies in this chamber
        self.framelength = DEFAULT_FRAMELENGTH # Time between each frame.

        # Interface and GraphWin will be set later:
        self.win = None
        self.interface = None

        # Add bodies to the chamber according to the given data.        
        self.updateChamberBodies( numbodies, gravangle,
                               gravaccel, airres, rad, bodycol )
        return

    def setInterface( self, interface ):
        """Setter for the chamber interface."""
        self.interface = interface
        return

    def updateChamberBodies( self, numbodies, gravangle, gravaccel,
                          airres, rad, bodycol ):
        """Modifies the FallingBody instances in the chamber
    according to the given options."""
        scalefactor = min( self.width, self.height ) * 0.001
        if numbodies >= len( self.bodies ):
            for i in range( numbodies - len( self.bodies ) ):
                self.addBody( self.width / 2,
                          self.height / 2 + i * scalefactor, 0, 0, rad,
                          bodycol, gravangle, gravaccel, airres)
            return
        else:
            for i in range( len( self.bodies ) - numbodies ):
                self.bodies[-i-1].undrawBody()
            self.bodies = self.bodies[:numbodies]
            return

    def exportDataToInterface( self ):
        """Exports the chamber data to the interface. Currently only
    gravity angle and acceleration can be modified outside of the interface,
    so we only worry about updating these."""
        angle = 0
        accel = 0
        if len( self.bodies ) > 0:
            angle = self.gravangle
            accel = self.gravaccel
        self.interface.setGravAngle( angle )
        self.interface.setGravAccel( accel )
        return
            
    def updateChamber( self, col, fric, numbodies,
                  gravangle, gravaccel, gravconst, rad, airres, bodycol ):
        """Updates the chamber variables according to the given input."""
        self.interface.giveValueWarnings( col, fric,
                  gravaccel, gravconst, rad, airres, bodycol )
        
        self.setCol( col )
        self.setFric( fric )
        
        self.gravangle = gravangle
        self.gravaccel = gravaccel
        self.gravconst = gravconst
        self.tempgravangle = 0
        self.tempgravaccel = 0
        
        self.updateChamberBodies( numbodies, gravangle, gravaccel,
                          airres, rad, bodycol )
        self.redrawBodies()
        self.setGravity( gravangle, gravaccel )
        self.setAirRes( airres )
        self.setBodyCol( bodycol )
        return
    
    def drawChamber( self ):
        """Draws a GraphWin object containing the chamber."""
        self.win = GraphWin( "Gravity Chamber",
                             self.offset * self.width,
                             self.offset * self.height,
                             autoflush = False )
        
        self.win.setCoords( -( ( self.offset - 1 ) / 2 ) * self.width,
                            -( ( self.offset - 1 ) / 2 ) * self.height,
                            ( ( self.offset - 1 ) / 2 + 1 ) * self.width,
                            ( ( self.offset - 1 ) / 2 + 1 ) * self.height )                            
        Line( Point( 0, 0 ), Point( 0, self.height ) ).draw( self.win )
        Line( Point( 0, self.height ),
              Point( self.width, self.height ) ).draw( self.win )
        Line( Point( self.width, self.height ),
              Point( self.width, 0 ) ).draw( self.win )
        Line( Point( self.width, 0 ), Point( 0, 0 ) ).draw( self.win )
        return

    def drawBodies( self ):
        """Draws each FallingBody in the chamber."""
        for body in self.bodies:
            body.drawBody()
        return

    def undrawBodies( self ):
        """Undraws each FallingBody in the chamber."""
        for body in self.bodies:
            body.undrawBody()
        return

    def redrawBodies( self ):
        """Redraws each FallingBody in the chamber."""
        self.undrawBodies()
        self.drawBodies()
        return

    #def setGravAngle( self, angle ):
        #"""Setter for the angle of gravity."""
        #self.gravangle = angle
        #for body in self.bodies:
        #    body.setGravAngle( angle )
        #return

    #def setGravAccel( self, accel ):
        #"""Setter for the acceleration of gravity."""
        #self.gravaccel = accel
        #for body in self.bodies:
        #    body.setGravAccel( accel )
        #return

    def setGravity( self, direction, accel ):
        """Sets the direction and acceleration of gravity in the chamber."""
        self.gravangle = direction
        self.gravaccel = accel
        for body in self.bodies:
            body.setGravity( direction, accel )
        return

    #def applyZeroGravity( self ):
    #    for body in self.bodies:
    #        body.setGravity( 0, 0 )
    #    return

    def setAirRes( self, airres ):
        """Setter for chamber air resistance."""
        for body in self.bodies:
            body.setAirRes( airres )
        return

    def setBodyCol( self, bodycol ):
        """Setter for body collision coefficient."""
        for body in self.bodies:
            body.setBodyCol( bodycol )
        return

    def setCol( self, col ):
        """Setter for wall collision coefficient."""
        self.col = col
        return

    def setFric( self, fric ):
        """Setter for wall friction coefficient."""
        self.fric = fric
        return

    """def printIntro( self ):
        introText = Text(
            Point( self.width/2, self.height/2 ),
                "\n".join(["Use the arrows to change the direction of gravity,",
                "or press spacebar to turn gravity off.",
                "Press 'q' to stop the simulation.",
                "", "Press any key to begin."]) )
        #introBox = Rectangle( Point
        introText.draw( self.win )
        self.interface.getWin().getKey()        
        introText.undraw()

    def printOutro( self ):
        Prints an outro text.
        outroText = Text(
            Point( self.width/2, self.height/2 ),
                    "Press any key to quit." )
        outroText.draw( self.win )
        self.interface.getWin().getKey()        
        outroText.undraw()"""        

    def getWin( self ):
        """Returns GraphWin object associated to the chamber."""
        return self.win

    def getHeight( self ):
        """Returns height of the chamber."""
        return self.height

    def getWidth( self ):
        """Return width of the chamber."""
        return self.width

    def getCol( self ):
        """Returns collision coefficient of chamber walls.
        (0 = inelastic, 1 = elastic) """
        return self.col

    def getFric( self ):
        """Returns friction coefficient of chamber walls.
        (0 = no friction, 1 = all friction )"""
        return self.fric

    def getBodies( self ):
        """Returns array of FallingBodies in the chamber."""
        return self.bodies

    def addBody( self, xpos, ypos, xvel, yvel, rad, col,
                 gravangle, gravaccel, res):
        """Adds a FallingBody to the chamber."""
        self.bodies.append(
            FallingBody( xpos, ypos, xvel, yvel, rad,
                         col, gravangle, gravaccel, res, self ) )
        return

    def displayHelp( self ):
        helpInterface = HelpInterface( "Help", 400, 400 )
        helpInterface.getInputAndClose()
        return

    def runChamber( self ):
        """Runs the gravity chamber until 'q' is pressed,
    and allows the user to change the direction of gravity
    with the arrow keys."""
        self.drawBodies()        
        while True:
            # At the start of each frame, color all bodies green
            # by default if in DEBUG mode.
            if DEBUG:
                for body in self.bodies:
                    body.setColor( body.getColor() )
                    
            keypressed = self.interface.getWin().checkKey()
            clicked = self.interface.getWin().checkMouse()
            action = self.interface.getAction( clicked )

            # Exit if close was selected or 'q' was pressed:
            if ( action == "Close" ) or ( keypressed in ['q', 'Q'] ):
                break

            # Display help message if help was selected:
            elif action == "Help":
                self.displayHelp()

            # Apply changes if apply was selected:
            elif ( action == "Apply" ) or ( keypressed == "Return" ):
                try:
                    self.updateChamber( self.interface.getWallCol(),
                                           self.interface.getWallFric(),
                                           self.interface.getNumberOfBodies(),
                                           self.interface.getGravAngle(),
                                           self.interface.getGravAccel(),
                                           self.interface.getGravConst(),
                                           self.interface.getRad(),
                                           self.interface.getAirRes(),
                                           self.interface.getBodyCol() )
                except: # Catch type errors from user input.
                    #raise
                    self.interface.handleInputException()

                # Set zeroGravity flag if necessary
                if self.interface.getGravAccel() == 0:
                    self.zeroGravity = True
                else:
                    self.zeroGravity = False

            # Change the direction of gravity
            # if a direction is pressed:
            elif keypressed in ["Right", "Up", "Left", "Down"]:
                directions = { "Right": 0, "Up": pi/2, "Left": -pi, "Down": -pi/2 }
                self.setGravity( directions[keypressed], self.interface.getGravAccel() )
                self.exportDataToInterface()

            # Toggle gravity if space is pressed:
            elif keypressed == "space":   
                if not self.zeroGravity:
                    self.tempgravangle = self.gravangle
                    self.tempgravaccel = self.gravaccel
                    self.setGravity( 0, 0 )
                else:                
                    self.setGravity( self.tempgravangle, self.tempgravaccel )
                self.zeroGravity = not( self.zeroGravity )
                self.exportDataToInterface()
                
            # Handle any collisions between FallingBodies:
            self.handleCollisions()

            # Update the acceleration of the bodies due to gravity:
            self.updateGravities()

            for body in self.bodies:
                # Handle any wall collisions:
                body.handleWallCollisions( self.framelength )

                # Update the position of each body:
                body.update( self.framelength )

            # graphics.py update() method:
            update()

        # Close the chamber when the animation loop has finished.
        self.close()
        self.interface.close()
        return

    def updateGravities( self ):
        """ Uses the gravitional constant and relative positions to set
        acceleration for each FallingBody in the chamber."""

        # return immediately if no gravity between bodies
        if self.gravconst == 0:
            return

        # start with the acceleration of gravity
        xaccels = [ self.gravaccel * cos( self.gravangle ) ] * len( self.bodies )
        yaccels = [ self.gravaccel * sin( self.gravangle ) ] * len( self.bodies )

        # compute the acceleration between each pair of bodies
        for i in range( len( self.bodies ) - 1 ):
            for j in range( i + 1, len( self.bodies ) ):
                xposi = self.bodies[i].getX()
                yposi = self.bodies[i].getY()
                xposj = self.bodies[j].getX()
                yposj = self.bodies[j].getY()
                radi = self.bodies[i].getRad()
                radj = self.bodies[j].getRad()
                distsquared = ( xposi - xposj ) ** 2 + ( yposi - yposj ) ** 2
                if distsquared != 0:
                    accel = self.gravconst * radi**2 * radj**2 / (
                        distsquared )
                    xaccelchange = accel * ( xposj - xposi ) / sqrt(
                        distsquared )
                    yaccelchange = accel * ( yposj - yposi ) / sqrt(
                        distsquared )
                    xaccels[i] += xaccelchange
                    yaccels[i] += yaccelchange
                    xaccels[j] -= xaccelchange
                    yaccels[j] -= yaccelchange

        # modify acceleration for each pair
        for i in range( len( self.bodies ) ):
            self.bodies[i].setAccel( xaccels[i], yaccels[i] )
        return

    def handleCollisions( self ):
        """Updates the velocities of all colliding objects,
    and reset their positions if applicable. This function runs every
    frame so optimizing it later is important."""
        
        for i in range( len( self.bodies ) - 1 ):
            for j in range( i + 1, len( self.bodies ) ):
                # Compute the collision tolerance of these two bodies:
                tol = ( self.bodies[i].getTolerance( self.framelength ) +
                        self.bodies[j].getTolerance( self.framelength) )

                # Check if the bodies are touching within this
                # tolerance and proceed accordingly:
                
                if self.bodiesTouching( self.bodies[i],
                                        self.bodies[j], tol ):

                    # If one body is inside another, reset
                    # the position of the body which is
                    # more interior to the chamber:
                    bodyToReset = self.bodyToReset( i, j )                    
                    if bodyToReset == j:
                        self.resetPosOnCollision( self.bodies[i],
                                                  self.bodies[j] )
                    else:
                        self.resetPosOnCollision( self.bodies[j],
                                                  self.bodies[i] )
                    # Handle the collision:
                    self.applyCollision( self.bodies[i], self.bodies[j], tol )

        return

    def bodyToReset( self, i, j ):
        """Returns whichever body is farther away from the boundary, according
    to which one has maximal horizontal or vertical distance from the
    boundary."""

        xpos1 = self.bodies[i].getX()
        ypos1 = self.bodies[i].getY()
        xpos2 = self.bodies[j].getX()
        ypos2 = self.bodies[j].getY()
        r1 = self.bodies[i].getRad()
        r2 = self.bodies[j].getRad()
        
        body1_leftright = min( xpos1 - r1, self.width - ( xpos1 + r1 ) )
        body1_updown = min( ypos1 - r1, self.height - ( ypos1 + r1 ) )
        body2_leftright = min( xpos2 - r2, self.width - ( xpos2 + r2 ) )
        body2_updown = min( ypos2 - r2, self.height - ( ypos2 + r2 ) )

        if min( body1_leftright, body1_updown ) < min( body2_leftright,
                                                        body2_updown ):
            return j
        elif min( body1_leftright, body1_updown ) > min( body2_leftright,
                                                        body2_updown ):
            return i
        else:
            if max( body1_leftright, body1_updown ) < max( body2_leftright,
                                                        body2_updown ):
                return j
            else:
                return i       

    def resetPosOnCollision( self, body1, body2 ):
        """Move body1 and body2 to be exactly radius( body1 ),
    radius( body2 ) apart, assuming one is inside the other.
    In this case, fix the position of body1 and only move body2."""
        xdiff = 0
        ydiff= 0      

        xpos1 = body1.getX()
        ypos1 = body1.getY()
        xpos2 = body2.getX()
        ypos2 = body2.getY()
        
        # Calculate ideal collision distance between the two bodies:
        idistance = body1.getRad() + body2.getRad()

        # Calculate the square of the actual distance between
        # the two bodies:
        distance = sqrt( ( xpos2 - xpos1 ) ** 2 + ( ypos2 - ypos1 ) ** 2 )

        # Compute scale factor:
        scalefactor = 0
        if distance != 0:
            scalefactor = ( distance - idistance ) / distance

        # Return immediately if one body is not partially inside the other.
        if scalefactor >= 0:
            return

        xdiff = ( xpos2 - xpos1 ) * scalefactor
        ydiff = ( ypos2 - ypos1 ) * scalefactor

        # Smoothly move the FallingBody back in bounds
        # in self.correctionframes steps
        for i in range( body2.correctionframes ):
            body2.move( -xdiff / body2.correctionframes,
                          -ydiff / body2.correctionframes )

        # Color blue on correction if in DEBUG mode.
        if DEBUG:
            body2.setFill( "blue" )
        return      
        
                
    def bodiesTouching( self, body1, body2, tol ):
        """If body1 and body2 are touching within a tolerated amount,
    returns True. This function should be further optimized later."""

        # First, check relative velocities and positions to see if these
        # objects can even collide. Return False immediately if not.
        xpos1 = body1.getX()
        ypos1 = body1.getY()
        xpos2 = body2.getX()
        ypos2 = body2.getY()
        xvel1 = body1.getXvel()
        yvel1 = body1.getYvel()
        xvel2 = body2.getXvel()
        yvel2 = body2.getYvel()
        if ( ( ( xpos1 < xpos2 + tol ) and ( xvel1 < xvel2 ) or
            ( xpos1 + tol > xpos2 ) and ( xvel1  > xvel2 ) ) and
            ( ( ypos1 < ypos2 + tol ) and ( yvel1 < yvel2 ) or
            ( ypos1 + tol > ypos2 ) and ( yvel1 > yvel2 ) ) ):
            return False

        # Calculate ideal collision distance between the two bodies
        idistance = body1.getRad() + body2.getRad() 

        # Calculate the square of the actual distance between
        # the two bodies.
        dsquared = ( xpos2 - xpos1 ) ** 2 + ( ypos2 - ypos1 ) ** 2

        # Checks if the body1 is too close to or inside of body2:
        touching = sqrt( dsquared ) <= idistance + tol

        # Alternative, squareroot-free approach which only
        # determines if sqrt( dsquared ) and idistance are within
        # tol of each other:
        # touching = ( dsquared + idistance**2 - tol ** 2
        #             ) ** 2 <=  4 * dsquared * idistance**2

        # Color the bodies orange on collision if in DEBUG mode.
        if touching and DEBUG:
            body1.setFill( "orange" )
            body2.setFill( "orange" )
            
        return touching

    def applyCollision( self, body1, body2, tol ):
        """Updates the velocities of the
    colliding objects, but not their positions.
    Collisions are elastic for now. Uses "angle-free" elastic collision formula:
    (https://en.wikipedia.org/wiki/Elastic_collision#Two-dimensional_collision_with_two_moving_objects)"""

        # Store position, velocity, and radius information of both bodies.
        xpos1 = body1.getX()
        ypos1 = body1.getY()
        xpos2 = body2.getX()
        ypos2 = body2.getY()
        
        xvel1 = body1.getXvel()
        yvel1 = body1.getYvel()
        xvel2 = body2.getXvel()
        yvel2 = body2.getYvel()

        r1 = body1.getRad()
        r2 = body2.getRad()

        # Store collision coefficient information of both bodies.
        col1 = body1.getCol()
        col2 = body2.getCol()

        # Area/mass is proportional to radius squared,
        # and the pi cancels in the collision equation,
        # so it suffices to assume mass = radius**2
        m1 = r1**2
        m2 = r2**2

        # Calculate the coefficients in the collision equation.
        masscoeff1 = 2 * m2 / ( m1 + m2 )
        masscoeff2 = 2 * m1 / ( m1 + m2 )
        velcoeff = ( ( xvel1 - xvel2 ) * ( xpos1 - xpos2 ) +
                      ( yvel1 - yvel2 ) * ( ypos1 - ypos2 ) ) /  ( ( r1 + r2 )**2 )

        # Apply the elastic collision equation to compute the new velocity.
        # col1 and col2 are used to "artificially" make the
        # collision partially inelastic. This is definitely not the correct
        # way to handle partially inelastic collisions, but I am lazy.
        xvel1_new = ( xvel1 - masscoeff1 * velcoeff * ( xpos1 - xpos2 ) ) * col1
        yvel1_new = ( yvel1 - masscoeff1 * velcoeff * ( ypos1 - ypos2 ) ) * col1

        xvel2_new = xvel2 - masscoeff2 * velcoeff * ( xpos2 - xpos1 ) * col2
        yvel2_new = yvel2 - masscoeff2 * velcoeff * ( ypos2 - ypos1 ) * col2

        # Update the velocities of the two bodies after the collision.
        body1.setVel( xvel1_new, yvel1_new )
        body2.setVel( xvel2_new, yvel2_new )
        
        return        

    def close( self ):
        """Closes the gravity chamber."""
        self.win.close()
        return

class OptionMenu:
    """A graphical object consisting of a list of options to be
displayed in a graphics window."""

    charlimit = 6 # character limit for entry boxes
    distance = 0 # distance between label and entry box

    def __init__( self, labels, defaults, ycoord, percentage, win ):
        """Labels is a list of labels for the options. ycoord is the top of
    the option list. percentage is the size of the window this option menu
    will take up. Defaults is the default valu"""
        height = percentage * win.getHeight() / 100
        optionheight = height / len( labels )
        self.eBoxes = []
        for i in range( len( labels ) ):
            anchorPoint = Point( win.getWidth() / 2,
                optionheight * ( 1 / 2 + i ) + ycoord )
            textbox = Text( anchorPoint, labels[i] )
            textbox.move( 0, -( 12 + self.distance / 2 ) )
            textbox.draw( win )
            self.eBoxes.append( Entry( anchorPoint, self.charlimit ) )
            self.eBoxes[-1].move( 0, 12 + self.distance / 2 )
            self.eBoxes[-1].setText( defaults[i] )
            self.eBoxes[-1].setFill( "white" )
            self.eBoxes[-1].draw( win )
        return

    def getInput( self, argindex ):
        """Return the input from the Entry Box at the given index as a float."""
        return float( self.eBoxes[argindex].getText() )
    
class ButtonMenu:
    """A graphical object consisting of a list of buttons."""

    fillpercent = 95 # How large will the buttons be relative to the menu?

    def __init__( self, labels, ycoord, percentage, win ):
        """Labels is a list of labels for the buttons. ycoord is the top of
    the button list. percentage is the size of the window this button menu
    will take up."""
        height = percentage * win.getHeight() / 100
        self.buttons = []
        for i in range( len( labels ) ):
            self.buttons.append( Button( win, Point( win.getWidth() / 2,
                height / len( labels ) * ( 1 / 2 + i ) + ycoord ),
                self.fillpercent * win.getWidth() / 100,
                self.fillpercent * height / len( labels ) / 100, labels[i] ) )
            self.buttons[-1].activate()
        return

    def getButtons( self ):
        """Return the button list for this menu."""
        return self.buttons

class MenuMessage:
    """A graphical object consisting of a message which takes up
some portion of a graphics window."""

    def __init__( self, msg, ycoord, percentage, win ):
        """msg is the message to be displayed. ycoord is the top of the message
    space. percentage indicates how much of the window this message space will
    occupy."""
        anchorPoint = Point( win.getWidth() / 2,
                             ycoord + win.getHeight() * percentage / 200 )
        self.msg = Text( anchorPoint, msg )
        self.msg.draw( win )
        return

    def undraw( self ):
        """Undraw the message."""
        self.msg.undraw()
        return

    def setText( self, newmsg ):
        """Set the message text."""
        self.msg.setText( newmsg )
        return

    def setStyle( self, style ):
        """Set the style of the message text."""
        self.msg.setStyle( style )
        return

    def setTextColor( self, color ):
        """Set the color of the message text."""
        self.msg.setTextColor( color )
        return

            
class StaticInterface:
    """Class for getting static chamber options which cannot be changed without
creating a new GraphWin, such as height and width of the chamber. Can be
closed by pressing any key. Will also be used to display directions. Also
used for displaying help."""

    closeMessage = "Press <Enter> to continue..."
    errorMessage = "Error: Invalid input.\nPress any key to continue..."
    
    def __init__( self, wintitle, width, height, labels, defaults ):
        """wintitle, width, and height refer to GraphWin parameters. labels
    is a list of labels for the OptionMenu. Defaults is their default values."""
        self.win = GraphWin( wintitle, width, height )  
        self.options = OptionMenu( labels, defaults, 0, 50, self.win )
        self.msg = MenuMessage( self.closeMessage, .5 * height, 50, self.win )

    def getInput( self ):
        """Gets the input from the interface."""
        while self.win.checkKey() != "Return":
            pass
        return self.options.getInput( 0 ), self.options.getInput( 1 )

    def close( self ):
        """Close the interface."""
        self.win.close()

    def handleInputException( self ):
        """Handle input errors."""
        self.msg.setText( self.errorMessage )
        self.msg.setStyle( "bold" )
        self.msg.setTextColor( "red" )
        self.win.getKey()
        self.msg.setText( self.closeMessage )
        self.msg.setStyle( "normal" )
        self.msg.setTextColor( "black" )
        return  


class HelpInterface:
    """Class for displaying a window with help message which closes when
    Return is pressed. I am doing this as a text-based message right now
    since creatin a new window and then closing it does not allow for
    input from other windows."""

    #closeMessage = "Press <Enter> to continue..."

    helpMessage = "\n".join(["\nUse the input boxes to change chamber parameters.\n",
        "Press <Enter> or click 'Apply' to apply changes to the chamber.",
        "Click 'Help' to display this message.",
        "Press <q> or click 'Close' to stop the simulation and close the program.",
        "Use the arrows to change the direction of gravity,",
        "or press spacebar to turn gravity off.\n",
        "Parameter explanations:\n",
        "Number of bodies [0,infinity): Nonnegative integer representing the",
        "number of bodies in the chamber.\n",
        "Radius of new bodies (0,infinity): If the number of bodies is greater",
        "than the number of bodies in the chamber, this postive float is",
        "the radius of the bodies to be added.\n",
        "Wall collision coefficient [0,1]: Float in the closed interval [0,1]",
                             "representing the collision coefficient of the walls.",
                             "(0 = inelastic, 1 = elastic )\n",
        "Wall friction coefficient [0,1]: Float in the closed interval [0,1]",
                             "representing the friction coefficient of the walls.",
                             "(0 = no friction, 1 = all friction )\n",
        "Body collision coefficient [0,1]: Float in the closed interval [0,1]",
                        "representing the collision coefficient of the bodies.",
                        "(0 = inelastic, 1 = elastic ).",
                        "This is 'artificial' and should be kept close to 1.\n",
        "Gravity angle (degrees): Float representing the angle in degrees that the",
                             "force of gravity makes with the positive x-axis",
                             "measured counterclockwise.\n",
        "Gravity acceleration [0,infinity): Nonnegative float representing the",
                             "acceleration of gravity.\n",
        "Air resistance [0,infinity): Nonnegative float representing the",
                             "air resistance of the chamber."
        "Gravitational constant [0,infinity): Nonnegative float representing the",
                             "gravitational constant inside the chamber."
                             ])

    
    def __init__( self, wintitle, width, height ):
        """ wintitle, height, and width are GraphWin parameters. I did not end
    up using a GraphWin to display help because there is an anomoly with
    GraphWin objects whereby only the most recent window opened can check for
    and use user input. This is a text-based interface for now, pending
    a fix for this anomoly."""
        print( self.helpMessage )
        #self.win = GraphWin( wintitle, width, height )  
        #MenuMessage( self.helpMessage, 0, 80, self.win )
        #MenuMessage( self.closeMessage, .8 * height, 20, self.win )
        return

    def getInputAndClose( self ):
        """Useless for text-based help interface."""
    #    while self.win.checkKey() != "Return":
    #        pass
    #   self.win.close()
        return

class ChamberInterface:
    """GravityChamber interface. Used for interactively setting chamber
    parameters."""

    width = 300
    height = 800
    #argIndexes = [ 2, 3 ]

    optionLabels = [ "Number of bodies [0,infinity):",
                     "Radius of new bodies (0,infinity):",
                   "Wall collision coefficient [0,1]:",
                   "Wall friction coefficient [0,1]:",
                   "Body collision coefficient [0,1]:",
                   "Gravity angle (degrees):",
                   "Gravity acceleration [0,infinity):",
                   "Air resistance [0,infinity):",
                   "Gravitational constant [0,infinity):",]
    #defaults = [ DEFAULT_NUM_OF_BODIES,
    #             DEFAULT_RADIUS,
    ###                 DEFAULT_WALL_COLLISION_COEFFICIENT,
      #               DEFAULT_FRICTION,
      #               DEFAULT_BODY_COLLISION_COEFFICIENT,
       #              degrees(DEFAULT_GRAVITY_DIRECTION),
       #              DEFAULT_GRAVITY_ACCELERATION,
       #              DEFAULT_AIR_RESISTANCE ]

    buttonLabels = [ "Apply", "Help", "Close" ]

    introMessage = "Modify chamber options below."

    errorMessage = "Error: Invalid input.\nPress any key to continue..."
    
    
    def __init__( self, defaults ):
        """Defaults is a list of default chamber options."""
        self.defaults = defaults
        self.win = GraphWin( "Chamber Control Panel", self.width, self.height)
        self.msg = MenuMessage( self.introMessage, 0 , 10, self.win )
        self.options = OptionMenu( self.optionLabels, self.defaults,
                                   0.1 * self.height, 60, self.win )
        self.buttons = ButtonMenu( self.buttonLabels, 0.75 * self.height,
                                   25, self.win )
        return

    def getAction( self, pointclicked ):
        """Returns the label of whichever button was clicked."""
        if pointclicked is None:
            return "None"
        for i in range( len( self.buttons.getButtons() ) ):
            if self.buttons.getButtons()[i].clicked( pointclicked ):
                return self.buttons.getButtons()[i].getLabel()

    def handleInputException( self ):
        """Handle input errors."""
        self.msg.setText( self.errorMessage )
        self.msg.setStyle( "bold" )
        self.msg.setTextColor( "red" )
        self.win.getKey()
        self.msg.setText( self.introMessage )
        self.msg.setStyle( "normal" )
        self.msg.setTextColor( "black" )
        return

    def giveValueWarnings( self, col, fric, gravaccel, gravconst, rad,
                           airres, bodycol ):
        """Print a warning to the terminal for numeric chamber parameters which
    do not make physical sense, so that the user knows to expect strange
    behavior."""
        if not ( 0 <= col <= 1 ):
            print( """Warning: Wall collision coefficient should be a real
number in the closed interval [0,1]""" )
        if not ( 0 <= fric <= 1 ):
            print( """Warning: Wall friction coefficient should be a real
number in the closed interval [0,1]""" )
        if not ( 0 <= bodycol <= 1 ):
            print( """Warning: Body collision coefficient should be a real
number in the closed interval [0,1]""" )
        if not ( gravaccel >= 0 ):
            print( """Warning: Gravity acceleration should be a real number
in the interval [0, infinity)""" )
        if not ( airres >= 0 ):
            print( """Warning: Air resistance should be a real number
in the interval [0, infinity)""" )
        if not ( rad > 0 ):
            print( """Warning: Radius of new bodies should be a real number
in the interval (0, infinity)""" )
        if not ( gravconst >= 0 ):
            print( """Warning: Gravitational constant should be a real number
in the interval [0, infinity)""" )
        return

    def getNumberOfBodies( self ):
        """Returns the number of FallingBodies in the interface."""
        return int( self.options.getInput( 0 ) )

    def getRad( self ):
        """Returns the radius of new FallingBodies to be added to the chamber
    in the interface."""
        return self.options.getInput( 1 )

    def getWallCol( self ):
        """Returns the wall collision coefficient in the interface."""
        return self.options.getInput( 2 )

    def getWallFric( self ):
        """Return the wall friction coefficient in the interface."""
        return self.options.getInput( 3 )

    def getBodyCol( self ):
        """Return the FallingBody collision coefficient in the interface."""
        return self.options.getInput( 4 )

    def getGravAngle( self ):
        """Return the angle of gravity in the interface."""
        return radians( self.options.getInput( 5 ) )

    def setGravAngle( self, angle ):
        """Setter for angle of gravity in the interface."""
        self.options.eBoxes[ 5 ].setText( str( degrees( angle ) ) )
        return

    def getGravAccel( self ):
        """Return the acceleration of gravity in the interface."""
        return self.options.getInput( 6 )

    def setGravAccel( self, accel ):
        """Setter for acceleration of gravity in the interface."""
        self.options.eBoxes[ 6 ].setText( str( accel ) )
        return

    def getAirRes( self ):
        """Return the air resistance in the interface."""
        return self.options.getInput( 7 )

    def getGravConst( self ):
        """Return the gravitational constant in the interface."""
        return self.options.getInput( 8 )

    """def processInput( self ):
        pass"""

    def getWin( self ):
        """Return the window of the interface."""
        return self.win

    def close( self ):
        """Close the interface window."""
        self.win.close()
        return

def runCustomChamber():
    """ Runs a GravityChambe with user-specified options."""
    defaults = [ DEFAULT_NUM_OF_BODIES,
                     DEFAULT_RADIUS,
                     DEFAULT_WALL_COLLISION_COEFFICIENT,
                     DEFAULT_FRICTION,
                     DEFAULT_BODY_COLLISION_COEFFICIENT,
                     degrees(DEFAULT_GRAVITY_DIRECTION),
                     DEFAULT_GRAVITY_ACCELERATION,
                     DEFAULT_AIR_RESISTANCE,
                     DEFAULT_GRAVITATIONAL_CONSTANT]

    #Create an interface to get chamber width and height:"""
    dimInterface = StaticInterface( "Gravity Chamber Dimensions", 250, 200,
                    [ "Chamber width (0,infinity):",
                   "Chamber height (0,infinity):"],
                   [ DEFAULT_WIDTH, DEFAULT_HEIGHT ] )

    while True:
        try:
            height, width = dimInterface.getInput()
            if not ( ( height > 0 ) and ( width > 0 ) ):
                # Handle the error: The dimensions do not make sense.
                dimInterface.handleInputException()
                continue
            # Initialize the chamber using the default options:
            chamb = GravityChamber( height, width, DEFAULT_OFFSET,
                            DEFAULT_WALL_COLLISION_COEFFICIENT,
                            DEFAULT_FRICTION,
                            DEFAULT_NUM_OF_BODIES,
                            DEFAULT_GRAVITY_DIRECTION,
                            DEFAULT_GRAVITY_ACCELERATION,
                            DEFAULT_GRAVITATIONAL_CONSTANT,
                            DEFAULT_RADIUS,
                            DEFAULT_AIR_RESISTANCE,
                            DEFAULT_BODY_COLLISION_COEFFICIENT )
            dimInterface.close()
            break
        except:
            # Handle type errors
            dimInterface.handleInputException()

    # Draw the chamber:    
    chamb.drawChamber()

    # Set the interface:
    interface = ChamberInterface( defaults )
    chamb.setInterface( interface )

    # Run the animation loop, which will check for further user input:
    chamb.runChamber()

    return

def main():
    runCustomChamber()
    return
        
if __name__== "__main__": main()








###################################################
# SOME TESTS THAT I HAD USED FOR DEBUGGING FOLLOW #
###################################################



def runRandomChamber( numbodies, numlayers ):
    """Runs a gravity chamber with numbodies * numlayers
    FallingBodies in numlayers rows of numbodies each, randomizing
    their starting velocities and radii as well."""
    
    chamb = GravityChamber( DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_OFFSET,
                            DEFAULT_WALL_COLLISION_COEFFICIENT,
                            DEFAULT_FRICTION )

    randomRadius = True
    
    for i in range( numlayers ):
        for j in range( numbodies ):
            mindim = min( DEFAULT_WIDTH, DEFAULT_HEIGHT )
            maxradius = mindim / ( 4 * max( numbodies, numlayers ) )
            radius = maxradius
            if randomRadius:
                radius = ( 1 - 0.8 * random() ) * maxradius
            angle = random() * 2 * pi
            speed = 5 #random() * ( DEFAULT_WIDTH + DEFAULT_HEIGHT ) / sqrt(
                #(numbodies * numlayers) )
            chamb.addBody( mindim / ( 2 * numbodies ) + j *
                           ( mindim / numbodies ),
                           mindim / ( 2 * numlayers ) + i *
                           ( mindim / numlayers ),
                           speed * cos( angle ),
                           speed * sin( angle ), radius,
                           DEFAULT_BODY_COLLISION_COEFFICIENT,
                           DEFAULT_GRAVITY_DIRECTION,
                           DEFAULT_GRAVITY_ACCELERATION,
                           DEFAULT_AIR_RESISTANCE )

    chamb.runChamber()

def runDropTest( numbodies ):
    """A method which drops one ball directly on another."""
    chamb = GravityChamber( DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_OFFSET,
                            DEFAULT_WALL_COLLISION_COEFFICIENT, DEFAULT_FRICTION )
    
    mindim = min( DEFAULT_WIDTH, DEFAULT_HEIGHT )
    maxradius = mindim / ( numbodies * 4 )
    speed = 0
    for i in range( numbodies ):
        radius = maxradius # ( 1 - 0.4 * random() ) * maxradius
        chamb.addBody( mindim / 2,
                       mindim / ( 2 * numbodies ) + i *
                       ( mindim / numbodies ), 0, speed, radius,
                       DEFAULT_BODY_COLLISION_COEFFICIENT,
                       DEFAULT_GRAVITY_DIRECTION,
                       DEFAULT_GRAVITY_ACCELERATION,
                       DEFAULT_AIR_RESISTANCE )

    chamb.runChamber()


def runCollisionTest():
    """A method for debugging collision anomolies. Two bodies will be
    set in motion towards each other with an optional offset angle."""
    
    radius1 = 100
    radius2 = 50
    speed1 = 100
    speed2 = 100
    offset = 0
    
    chamb = GravityChamber( DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_OFFSET,
                            DEFAULT_WALL_COLLISION_COEFFICIENT,
                            DEFAULT_FRICTION )
    
    chamb.addBody( 1.1 * radius1, DEFAULT_HEIGHT / 2, speed1, offset, radius1,
                   DEFAULT_BODY_COLLISION_COEFFICIENT,
                   DEFAULT_GRAVITY_DIRECTION,
                   DEFAULT_GRAVITY_ACCELERATION,
                   DEFAULT_AIR_RESISTANCE )
    chamb.addBody( DEFAULT_WIDTH - 1.1 * radius2, DEFAULT_HEIGHT / 2, -speed2,
                   0, radius2, DEFAULT_BODY_COLLISION_COEFFICIENT,
                   DEFAULT_GRAVITY_DIRECTION,
                   DEFAULT_GRAVITY_ACCELERATION,
                   DEFAULT_AIR_RESISTANCE )

    #run the chamber:
    chamb.runChamber()

def runWallStickTest():
    """A method which demonstrates the bug that an object which is
    in collision with a wall and has very low velocity will "stick"
    to that wall, even if gravity is pulling it away. This arose when fixing
    the issue of objects moving through the walls as a result of gravity
    and should be addressed by explicitly checking if you are in collision
    with a wall which is behind you from the point of view of gravity."""
    chamb = GravityChamber( DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_OFFSET,
                            DEFAULT_WALL_COLLISION_COEFFICIENT,
                            DEFAULT_FRICTION )

    gravangle = -pi / 2 + pi / 4

    radius = 50
    distancefromwall = 0.16
    
    chamb.addBody( radius + distancefromwall, DEFAULT_HEIGHT / 2, 0, 0, radius,
                   DEFAULT_BODY_COLLISION_COEFFICIENT,
                   gravangle,
                   DEFAULT_GRAVITY_ACCELERATION,
                   DEFAULT_AIR_RESISTANCE )
    chamb.runChamber()

def runOverlappingBodies( n ):
    chamb = GravityChamber( DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_OFFSET,
                            DEFAULT_WALL_COLLISION_COEFFICIENT,
                            DEFAULT_FRICTION )

    #gravangle = -pi / 2 + pi / 4

    radius = 50

    for i in range( n ):    
        chamb.addBody( DEFAULT_WIDTH / 2 + i/7000, DEFAULT_HEIGHT / 2, 0, 0, radius,
                   DEFAULT_BODY_COLLISION_COEFFICIENT,
                   DEFAULT_GRAVITY_DIRECTION,
                   DEFAULT_GRAVITY_ACCELERATION,
                   DEFAULT_AIR_RESISTANCE )

    chamb.runChamber()
         
