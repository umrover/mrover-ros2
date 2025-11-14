const { test, expect } = require('@playwright/test');

test('basestation frontend loads', async ({ page }) => {
  await page.goto('http://localhost:8080/');

  // You can add more specific assertions here
  // For example, checking for a specific title
  await expect(page).toHaveTitle(/MRover/);
});
