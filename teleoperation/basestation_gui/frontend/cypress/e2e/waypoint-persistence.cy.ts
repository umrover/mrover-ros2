describe('Waypoint Persistence Test', () => {
  beforeEach(() => {
    cy.mockWaypointsAPI()
    cy.mockAutonAPI()
  })

  it('should persist waypoint and costmap state after page refresh', () => {
    const testWaypoint = {
      name: 'Test Waypoint',
      id: 1,
      type: 1,
      lat: 42.2935,
      lon: -83.7144,
      enable_costmap: true,
      in_route: false
    }

    cy.visit('/AutonTask')

    cy.wait('@getAutonWaypoints')
    cy.wait('@getCurrentCourse')

    cy.get('[data-testid="waypoint-store-0"]').click()

    cy.wait('@saveCurrentCourse').then((interception) => {
      const savedCourse = interception.request.body.course
      expect(savedCourse).to.have.length(1)
      expect(savedCourse[0]).to.have.property('enable_costmap', true)
    })

    cy.get('[data-testid="costmap-toggle"]').click()

    cy.wait('@saveCurrentCourse').then((interception) => {
      const savedCourse = interception.request.body.course
      expect(savedCourse).to.have.length(1)
      expect(savedCourse[0]).to.have.property('enable_costmap', false)
    })

    cy.intercept('GET', '/api/waypoints/auton/current/', (req) => {
      req.reply({
        statusCode: 200,
        body: {
          status: 'success',
          course: [{
            name: 'No Search 1',
            id: -1,
            type: 0,
            lat: 0,
            lon: 0,
            enable_costmap: false
          }]
        }
      })
    }).as('getCurrentCourseRefresh')

    cy.reload()

    cy.wait('@getCurrentCourseRefresh')

    cy.get('[data-testid="waypoint-item"]').should('exist')
    cy.get('[data-testid="waypoint-costmap-toggle"]').should('exist')

    cy.get('[data-testid="waypoint-costmap-toggle"]').within(() => {
      cy.get('.bi-square').should('exist')
      cy.get('.bi-check-square-fill').should('not.exist')
    })
  })

  it('should add waypoint with correct costmap state based on All Costmaps toggle', () => {
    cy.visit('/AutonTask')

    cy.wait('@getAutonWaypoints')
    cy.wait('@getCurrentCourse')

    cy.get('[data-testid="costmap-toggle"]').click()

    cy.wait('@saveCurrentCourse')

    cy.get('[data-testid="waypoint-store-0"]').click()

    cy.wait('@saveCurrentCourse').then((interception) => {
      const savedCourse = interception.request.body.course
      expect(savedCourse).to.have.length(1)
      expect(savedCourse[0]).to.have.property('enable_costmap', false)
    })
  })

  it('should toggle all costmaps in current course', () => {
    cy.intercept('GET', '/api/waypoints/auton/current/', {
      statusCode: 200,
      body: {
        status: 'success',
        course: [
          { name: 'WP1', id: 1, type: 0, lat: 0, lon: 0, enable_costmap: true },
          { name: 'WP2', id: 2, type: 0, lat: 0, lon: 0, enable_costmap: true },
          { name: 'WP3', id: 3, type: 0, lat: 0, lon: 0, enable_costmap: true }
        ]
      }
    }).as('getMultipleWaypoints')

    cy.visit('/AutonTask')

    cy.wait('@getMultipleWaypoints')

    cy.get('[data-testid="waypoint-item"]').should('have.length', 3)

    cy.get('[data-testid="costmap-toggle"]').click()

    cy.wait('@saveCurrentCourse').then((interception) => {
      const savedCourse = interception.request.body.course
      expect(savedCourse).to.have.length(3)
      savedCourse.forEach((wp: any) => {
        expect(wp.enable_costmap).to.equal(false)
      })
    })
  })
})
