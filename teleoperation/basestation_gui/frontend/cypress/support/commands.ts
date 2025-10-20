declare global {
  namespace Cypress {
    interface Chainable {
      mockAutonAPI(): Chainable<void>
      mockWaypointsAPI(): Chainable<void>
    }
  }
}

Cypress.Commands.add('mockAutonAPI', () => {
  cy.intercept('POST', '/api/auton/enable/', {
    statusCode: 200,
    body: {
      status: 'success',
      enabled: true,
      waypoint_count: 1
    }
  }).as('enableAuton')
})

Cypress.Commands.add('mockWaypointsAPI', () => {
  cy.intercept('GET', '/api/waypoints/auton/', {
    statusCode: 200,
    body: {
      status: 'success',
      waypoints: []
    }
  }).as('getAutonWaypoints')

  cy.intercept('POST', '/api/waypoints/auton/save/', {
    statusCode: 200,
    body: {
      status: 'success'
    }
  }).as('saveAutonWaypoints')

  cy.intercept('GET', '/api/waypoints/auton/current/', {
    statusCode: 200,
    body: {
      status: 'success',
      course: []
    }
  }).as('getCurrentCourse')

  cy.intercept('POST', '/api/waypoints/auton/current/save/', (req) => {
    req.reply({
      statusCode: 200,
      body: {
        status: 'success',
        course: req.body.course
      }
    })
  }).as('saveCurrentCourse')
})

export {}
