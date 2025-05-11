// Helper function to update image with data that has MIME type
function updateImageWithMimeType(imgElement, dataString) {
  // Check if the data contains a MIME type
  if (dataString.includes(",")) {
    // Format: "mime_type,base64data"
    const [mimeType, base64data] = dataString.split(",", 2);
    imgElement.src = `data:${mimeType};base64,${base64data}`;
  } else {
    // Legacy format (fallback): assume JPEG if no MIME type provided
    imgElement.src = `data:image/jpeg;base64,${dataString}`;
  }

  // Optional: Update the decoding attribute for better performance
  imgElement.decoding = "async";

  // Optional: Add loading=lazy for images that might be off-screen
  if (!imgElement.hasAttribute("loading")) {
    imgElement.loading = "lazy";
  }
}
